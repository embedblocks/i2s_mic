/*
 * i2s_mic — implementation.
 *
 * See include/i2s_mic.h and the design/implementation-specification
 * documents for the contract this file implements. Section references in
 * comments below refer to the implementation specification unless noted
 * otherwise.
 *
 * Two distinct synchronization primitives are used, and must not be
 * confused (spec Section 2):
 *
 *   1. `s_mic.lifecycle_mutex` — an ordinary FreeRTOS mutex guarding the
 *      public lifecycle state and its transitions. Only ever touched from
 *      task context (init/start/stop/deinit are never called from an ISR).
 *
 *   2. `s_mic.spinlock` — a dual-context-safe critical section
 *      (portENTER_CRITICAL_SAFE / portENTER_CRITICAL_ISR) guarding
 *      `is_running`, `stopping`, `in_flight`, the overflow counters, and
 *      the pending-buffer queue's own bookkeeping. This is touched from
 *      both ISR and task context.
 */
#include <string.h>
#include <stdlib.h>

#include "i2s_mic.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "i2s_mic";

typedef enum {
    MIC_STATE_UNINITIALIZED = 0,
    MIC_STATE_INITIALIZED,
    MIC_STATE_RUNNING,
} mic_lifecycle_state_t;

typedef struct {
    uint8_t *buffer;
    size_t len;
} pending_slot_t;

typedef struct {
    /* --- Lifecycle (guarded by lifecycle_mutex; task context only) --- */
    SemaphoreHandle_t lifecycle_mutex;
    mic_lifecycle_state_t state;

    /* Set once by init() after everything below is fully constructed;
     * cleared once by deinit() before it frees anything. Deliberately an
     * ordinary, unsynchronized flag — see spec Section 2's note on why
     * request_buffer()'s UNINITIALIZED check must not use either lock. */
    volatile bool initialized_flag;

    /* --- ISR-safe state (guarded by spinlock) --- */
    portMUX_TYPE spinlock;
    volatile bool is_running;
    volatile bool stopping;
    volatile int in_flight;

    pending_slot_t *queue;   /* fixed-size array, allocated once at init() */
    int queue_cap;           /* == max_pending_buffers; fixed after init() */
    int queue_head;
    int queue_count;

    uint32_t total_overflow_count;
    uint32_t total_no_buffer_count;

    /* --- Effectively read-only after init(), before deinit() --- */
    size_t dma_buffer_size;
    mic_buffer_ready_cb_t cb;
    mic_overflow_cb_t overflow_cb;
    void *user_ctx;
    i2s_chan_handle_t rx_handle;
} i2s_mic_ctx_t;

static i2s_mic_ctx_t s_mic = {
    .spinlock = portMUX_INITIALIZER_UNLOCKED,
};

/* Forward declarations deliberately omit IRAM_ATTR: applying the attribute
 * to both a forward declaration and its definition creates two distinct
 * IRAM section-placement requests for the same symbol, which GCC rejects
 * under -Werror=attributes. IRAM_ATTR is applied once, on the definitions
 * below, which is sufficient to place the functions in IRAM. */
static bool mic_on_recv(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx);
static bool mic_on_recv_q_ovf(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx);

/* ------------------------------------------------------------------------
 * init / deinit / start / stop
 * ---------------------------------------------------------------------- */

esp_err_t i2s_mic_init(i2s_mic_config_t *config)
{
    if (config == NULL || config->cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->bits_per_sample != 16 && config->bits_per_sample != 32) {
        return ESP_ERR_INVALID_ARG;
    }
    bool slot_mode_ok =
        (config->channel_count == 1 && config->slot_mode == I2S_SLOT_MODE_MONO) ||
        (config->channel_count == 2 && config->slot_mode == I2S_SLOT_MODE_STEREO);
    if (!slot_mode_ok) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->max_pending_buffers < 1 || config->max_pending_buffers > 16) {
        return ESP_ERR_INVALID_ARG;
    }

    /* slot_bit_width == bits_per_sample for the 16/32-bit cases this
     * component supports (implementation spec Section 6/7). */
    const int slot_bit_width = config->bits_per_sample;
    const int bytes_per_frame = config->channel_count * (slot_bit_width / 8);
    if (config->dma_buffer_size <= 0 || (config->dma_buffer_size % bytes_per_frame) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const int dma_frame_num = config->dma_buffer_size / bytes_per_frame;

    if (s_mic.lifecycle_mutex == NULL) {
        s_mic.lifecycle_mutex = xSemaphoreCreateMutex();
        if (s_mic.lifecycle_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    xSemaphoreTake(s_mic.lifecycle_mutex, portMAX_DELAY);

    if (s_mic.state != MIC_STATE_UNINITIALIZED) {
        xSemaphoreGive(s_mic.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    pending_slot_t *queue = calloc((size_t)config->max_pending_buffers, sizeof(pending_slot_t));
    if (queue == NULL) {
        xSemaphoreGive(s_mic.lifecycle_mutex);
        return ESP_ERR_NO_MEM;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(config->port, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = config->dma_buffer_count;
    chan_cfg.dma_frame_num = dma_frame_num;

    i2s_chan_handle_t rx_handle = NULL;
    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret));
        free(queue);
        xSemaphoreGive(s_mic.lifecycle_mutex);
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            (i2s_data_bit_width_t)config->bits_per_sample, config->slot_mode),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = config->gpio_bck,
            .ws = config->gpio_ws,
            .dout = I2S_GPIO_UNUSED,
            .din = config->gpio_data,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        free(queue);
        xSemaphoreGive(s_mic.lifecycle_mutex);
        return ret;
    }

    i2s_event_callbacks_t cbs = {
        .on_recv = mic_on_recv,
        .on_recv_q_ovf = mic_on_recv_q_ovf,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };
    ret = i2s_channel_register_event_callback(rx_handle, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_register_event_callback failed: %s", esp_err_to_name(ret));
        i2s_del_channel(rx_handle);
        free(queue);
        xSemaphoreGive(s_mic.lifecycle_mutex);
        return ret;
    }

    s_mic.queue = queue;
    s_mic.queue_cap = config->max_pending_buffers;
    s_mic.queue_head = 0;
    s_mic.queue_count = 0;
    s_mic.dma_buffer_size = (size_t)config->dma_buffer_size;
    s_mic.cb = config->cb;
    s_mic.overflow_cb = config->overflow_cb;
    s_mic.user_ctx = config->user_ctx;
    s_mic.rx_handle = rx_handle;
    s_mic.is_running = false;
    s_mic.stopping = false;
    s_mic.in_flight = 0;
    s_mic.total_overflow_count = 0;
    s_mic.total_no_buffer_count = 0;

    s_mic.state = MIC_STATE_INITIALIZED;
    /* Set last, after every other field above is fully constructed —
     * request_buffer() reads this without a lock (spec Section 2). */
    s_mic.initialized_flag = true;

    xSemaphoreGive(s_mic.lifecycle_mutex);
    return ESP_OK;
}

esp_err_t i2s_mic_deinit(void)
{
    if (s_mic.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mic.lifecycle_mutex, portMAX_DELAY);

    if (s_mic.state != MIC_STATE_INITIALIZED) {
        xSemaphoreGive(s_mic.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    /* Cleared before anything is freed — see spec Section 2. */
    s_mic.initialized_flag = false;

    esp_err_t ret = i2s_del_channel(s_mic.rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_del_channel failed: %s", esp_err_to_name(ret));
    }

    free(s_mic.queue);
    s_mic.queue = NULL;
    s_mic.queue_cap = 0;
    s_mic.queue_head = 0;
    s_mic.queue_count = 0;
    s_mic.rx_handle = NULL;
    s_mic.cb = NULL;
    s_mic.overflow_cb = NULL;
    s_mic.user_ctx = NULL;
    s_mic.dma_buffer_size = 0;

    s_mic.state = MIC_STATE_UNINITIALIZED;

    xSemaphoreGive(s_mic.lifecycle_mutex);
    return ret;
}

esp_err_t i2s_mic_start(void)
{
    if (s_mic.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_mic.lifecycle_mutex, portMAX_DELAY);

    if (s_mic.state != MIC_STATE_INITIALIZED) {
        xSemaphoreGive(s_mic.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    /* Reset per-run counters and mark running before enabling the channel,
     * so the ISR never observes a stale/previous run's counts (spec
     * Section 9's reset behavior). */
    portENTER_CRITICAL(&s_mic.spinlock);
    s_mic.total_overflow_count = 0;
    s_mic.total_no_buffer_count = 0;
    s_mic.in_flight = 0;
    s_mic.is_running = true;
    portEXIT_CRITICAL(&s_mic.spinlock);

    esp_err_t ret = i2s_channel_enable(s_mic.rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(ret));
        portENTER_CRITICAL(&s_mic.spinlock);
        s_mic.is_running = false;
        portEXIT_CRITICAL(&s_mic.spinlock);
        xSemaphoreGive(s_mic.lifecycle_mutex);
        return ret;
    }

    s_mic.state = MIC_STATE_RUNNING;

    xSemaphoreGive(s_mic.lifecycle_mutex);
    return ESP_OK;
}

esp_err_t i2s_mic_stop(void)
{
    if (s_mic.lifecycle_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Held for stop()'s entire execution — serializes against a second
     * concurrent stop()/start() (spec Section 2/5). */
    xSemaphoreTake(s_mic.lifecycle_mutex, portMAX_DELAY);

    if (s_mic.state != MIC_STATE_RUNNING) {
        xSemaphoreGive(s_mic.lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    /* Close the gate: no RX callback dispatch that checks is_running after
     * this point can proceed, and request_buffer() starts rejecting via
     * `stopping`. */
    portENTER_CRITICAL(&s_mic.spinlock);
    s_mic.is_running = false;
    s_mic.stopping = true;
    portEXIT_CRITICAL(&s_mic.spinlock);

    esp_err_t ret = i2s_channel_disable(s_mic.rx_handle);
    if (ret != ESP_OK) {
        /* Not specified by the design/implementation documents. We still
         * complete the wait + queue-clear below so the component's internal
         * state stays consistent and future start()/stop() calls behave
         * correctly, but we surface this esp_err_t to the caller unchanged
         * per the general error-propagation rule (spec Section 11). */
        ESP_LOGE(TAG, "i2s_channel_disable failed: %s", esp_err_to_name(ret));
    }

    /* Wait for every in-flight RX callback dispatch (on_recv or
     * on_recv_q_ovf, including one currently inside the application's own
     * callback) to finish. Unbounded wait, yielding between polls — spec
     * Section 5, step 5. */
    int count;
    do {
        portENTER_CRITICAL(&s_mic.spinlock);
        count = s_mic.in_flight;
        portEXIT_CRITICAL(&s_mic.spinlock);
        if (count != 0) {
            vTaskDelay(1);
        }
    } while (count != 0);

    /* Single atomic critical-section entry: clear the queue and drop
     * `stopping` together (spec Section 5, step 6). */
    portENTER_CRITICAL(&s_mic.spinlock);
    s_mic.queue_head = 0;
    s_mic.queue_count = 0;
    s_mic.stopping = false;
    portEXIT_CRITICAL(&s_mic.spinlock);

    /* Public state transition, guarded by the lifecycle mutex alone. */
    s_mic.state = MIC_STATE_INITIALIZED;

    xSemaphoreGive(s_mic.lifecycle_mutex);
    return ret;
}

/* ------------------------------------------------------------------------
 * i2s_mic_request_buffer — dual-context-safe (task or ISR)
 * ---------------------------------------------------------------------- */

esp_err_t IRAM_ATTR i2s_mic_request_buffer(uint8_t *buffer, size_t buffer_len)
{
    /* Unsynchronized read by design — see spec Section 2. */
    if (!s_mic.initialized_flag) {
        return ESP_ERR_INVALID_STATE;
    }
    if (buffer == NULL || buffer_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    /* dma_buffer_size / queue_cap are fixed for the lifetime of a given
     * INITIALIZED/RUNNING period and only change under init()/deinit(),
     * which the caller must not invoke concurrently with this call on the
     * same instance (spec Section 2) — reading them here unsynchronized is
     * therefore safe. */
    if (buffer_len < s_mic.dma_buffer_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    esp_err_t ret;

    portENTER_CRITICAL_SAFE(&s_mic.spinlock);
    if (s_mic.stopping) {
        ret = ESP_ERR_INVALID_STATE;
    } else {
        bool dup = false;
        for (int i = 0; i < s_mic.queue_count; i++) {
            int idx = (s_mic.queue_head + i) % s_mic.queue_cap;
            if (s_mic.queue[idx].buffer == buffer) {
                dup = true;
                break;
            }
        }
        if (dup) {
            ret = ESP_ERR_INVALID_ARG;
        } else if (s_mic.queue_count >= s_mic.queue_cap) {
            ret = ESP_ERR_NO_MEM;
        } else {
            int tail = (s_mic.queue_head + s_mic.queue_count) % s_mic.queue_cap;
            s_mic.queue[tail].buffer = buffer;
            s_mic.queue[tail].len = buffer_len;
            s_mic.queue_count++;
            ret = ESP_OK;
        }
    }
    portEXIT_CRITICAL_SAFE(&s_mic.spinlock);

    return ret;
}

/* ------------------------------------------------------------------------
 * ISR callbacks
 * ---------------------------------------------------------------------- */

static bool IRAM_ATTR mic_on_recv(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    (void)handle;
    (void)user_ctx;

    uint8_t *dst = NULL;
    size_t dst_len = 0;
    bool have_buffer = false;
    uint32_t ovf_snapshot = 0;
    uint32_t nobuf_snapshot = 0;

    portENTER_CRITICAL_ISR(&s_mic.spinlock);
    if (!s_mic.is_running) {
        portEXIT_CRITICAL_ISR(&s_mic.spinlock);
        return false;
    }

    if (s_mic.queue_count > 0) {
        pending_slot_t slot = s_mic.queue[s_mic.queue_head];
        s_mic.queue_head = (s_mic.queue_head + 1) % s_mic.queue_cap;
        s_mic.queue_count--;
        dst = slot.buffer;
        dst_len = slot.len;
        have_buffer = true;
    } else {
        s_mic.total_no_buffer_count++;
    }
    ovf_snapshot = s_mic.total_overflow_count;
    nobuf_snapshot = s_mic.total_no_buffer_count;
    s_mic.in_flight++;
    portEXIT_CRITICAL_ISR(&s_mic.spinlock);

    if (have_buffer) {
        size_t copy_len = (event->size < dst_len) ? event->size : dst_len;
        /* ESP-IDF renamed this field from `data` to `dma_buf` (the old name
         * is gone entirely as of v6.0, not just deprecated). */
        memcpy(dst, event->dma_buf, copy_len);
        s_mic.cb(dst, copy_len, s_mic.user_ctx);
    } else if (s_mic.overflow_cb) {
        s_mic.overflow_cb(ovf_snapshot, nobuf_snapshot, s_mic.user_ctx);
    }

    portENTER_CRITICAL_ISR(&s_mic.spinlock);
    s_mic.in_flight--;
    portEXIT_CRITICAL_ISR(&s_mic.spinlock);

    return false;
}

static bool IRAM_ATTR mic_on_recv_q_ovf(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    (void)handle;
    (void)event;
    (void)user_ctx;

    uint32_t ovf_snapshot;
    uint32_t nobuf_snapshot;

    portENTER_CRITICAL_ISR(&s_mic.spinlock);
    if (!s_mic.is_running) {
        portEXIT_CRITICAL_ISR(&s_mic.spinlock);
        return false;
    }
    s_mic.total_overflow_count++;
    ovf_snapshot = s_mic.total_overflow_count;
    nobuf_snapshot = s_mic.total_no_buffer_count;
    s_mic.in_flight++;
    portEXIT_CRITICAL_ISR(&s_mic.spinlock);

    if (s_mic.overflow_cb) {
        s_mic.overflow_cb(ovf_snapshot, nobuf_snapshot, s_mic.user_ctx);
    }

    portENTER_CRITICAL_ISR(&s_mic.spinlock);
    s_mic.in_flight--;
    portEXIT_CRITICAL_ISR(&s_mic.spinlock);

    return false;
}
