/*
 * i2s_mic — implementation.
 *
 * The component is a thin wrapper around ESP-IDF's i2s_channel_read().
 * All buffering is the driver's own: finished DMA buffers wait in the
 * driver's internal queue (dma_buffer_count - 1 entries) until the
 * application reads them. The only ISR code here is a one-line counter in
 * on_recv_q_ovf, which the driver calls when that queue is full and the
 * oldest unread buffer is discarded, i.e. real audio loss.
 *
 * Driver behaviour this file depends on (ESP-IDF release/v6.0,
 * components/esp_driver_i2s/i2s_common.c):
 *
 *   1. i2s_channel_read() takes a binary semaphore, then loops while the
 *      channel state is RUNNING. If i2s_channel_disable() changes the state
 *      mid-read, the loop ends and the function returns ESP_OK with fewer
 *      bytes than requested (possibly 0). It returns ESP_ERR_TIMEOUT only
 *      when its queue wait expires, and ESP_ERR_INVALID_STATE when it cannot
 *      take the semaphore within the timeout.
 *   2. i2s_channel_disable() sets the state to READY, then waits (forever)
 *      for an in-progress read to release the semaphore, and keeps it.
 *      Only i2s_channel_enable() gives it back. A read that starts after
 *      disable() has finished therefore blocks until the next enable(), or
 *      until its own timeout.
 *   3. i2s_channel_enable() empties the RX queue.
 *   4. One DMA buffer is capped (4092 bytes on most targets). A larger
 *      request is silently shrunk, with only a log warning.
 *
 * How i2s_mic_read() stays correct while stop() runs on another task:
 *
 *   - Point 1: the result is checked after the call. ESP_OK only counts as
 *     success when the full buffer was copied; a short ESP_OK means the
 *     channel was stopped.
 *   - Point 2: the caller's timeout is never passed to the driver directly.
 *     The read waits in slices of about two buffer periods and re-checks
 *     s_mic.running between slices. stop() clears running before it
 *     disables the channel, so a reader always returns within one slice of
 *     stop(), whatever timeout it asked for.
 *   - Each read is exactly one DMA buffer. The driver copies one buffer at a
 *     time and only times out before copying, so a timed-out slice has
 *     consumed nothing and is safe to retry, and no partial buffer is ever
 *     left half-delivered.
 *
 * No lock is held across the driver call, so stop() is never blocked by
 * this wrapper; it only waits for the driver's own in-progress copy.
 */
#include <string.h>

#include "i2s_mic.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "driver/i2s_common.h"

static const char *TAG = "i2s_mic";

/* Lower bound for one wait slice, so very short DMA buffers don't turn the
 * read loop into a busy poll. */
#define I2S_MIC_MIN_SLICE_MS 20

typedef enum {
    MIC_STATE_UNINITIALIZED = 0,
    MIC_STATE_INITIALIZED,
    MIC_STATE_RUNNING,
} mic_state_t;

typedef struct {
    /* Guards `state` transitions to/from UNINITIALIZED and `reader_active`,
     * so deinit() and a starting read can't interleave. */
    portMUX_TYPE mux;

    volatile mic_state_t state;
    volatile bool running;        /* cleared by stop() before disabling */
    volatile bool reader_active;  /* a task is inside i2s_mic_read() */
    volatile uint32_t overflow_count;

    i2s_chan_handle_t rx;
    size_t buf_size;              /* actual bytes per DMA buffer */
    uint32_t slice_ms;            /* one wait slice inside i2s_mic_read() */
} i2s_mic_ctx_t;

static i2s_mic_ctx_t s_mic = {
    .mux = portMUX_INITIALIZER_UNLOCKED,
    .state = MIC_STATE_UNINITIALIZED,
};

/* Called by the driver from its RX ISR when its queue of finished buffers
 * is full and it has just discarded the oldest one. That buffer was never
 * read: one buffer of real audio loss. */
static bool IRAM_ATTR mic_on_recv_q_ovf(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    (void)handle;
    (void)event;
    (void)user_ctx;
    s_mic.overflow_count++;
    return false;
}

/* ------------------------------------------------------------------------
 * Lifecycle
 * ---------------------------------------------------------------------- */

esp_err_t i2s_mic_init(const i2s_mic_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->bits_per_sample != 16 && config->bits_per_sample != 32) {
        return ESP_ERR_INVALID_ARG;
    }
    bool slot_mode_ok =
        (config->channel_count == 1 && config->slot_mode == I2S_SLOT_MODE_MONO) ||
        (config->channel_count == 2 && config->slot_mode == I2S_SLOT_MODE_STEREO);
    if (!slot_mode_ok || config->sample_rate <= 0 || config->dma_buffer_count < 2) {
        return ESP_ERR_INVALID_ARG;
    }
    const int bytes_per_frame = config->channel_count * (config->bits_per_sample / 8);
    if (config->dma_buffer_size <= 0 || (config->dma_buffer_size % bytes_per_frame) != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const int frames = config->dma_buffer_size / bytes_per_frame;

    if (s_mic.state != MIC_STATE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(config->port, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = (uint32_t)config->dma_buffer_count;
    chan_cfg.dma_frame_num = (uint32_t)frames;

    i2s_chan_handle_t rx = NULL;
    i2s_chan_info_t info;
    size_t actual = 0;
    uint32_t period_ms = 0;
    uint32_t slice_ms = 0;
    const i2s_event_callbacks_t cbs = {
        .on_recv = NULL,
        .on_recv_q_ovf = mic_on_recv_q_ovf,
        .on_sent = NULL,
        .on_send_q_ovf = NULL,
    };

    esp_err_t ret = i2s_new_channel(&chan_cfg, NULL, &rx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)config->sample_rate),
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

    ret = i2s_channel_init_std_mode(rx, &std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    /* The driver silently shrinks a DMA buffer that exceeds its cap. Check
     * what it actually allocated, so every read is exactly one buffer. */
    ret = i2s_channel_get_info(rx, &info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_get_info failed: %s", esp_err_to_name(ret));
        goto fail;
    }
    actual = info.total_dma_buf_size / (uint32_t)config->dma_buffer_count;
    if (actual != (size_t)config->dma_buffer_size) {
        ESP_LOGE(TAG, "dma_buffer_size %d is too large for one DMA buffer; "
                 "the driver allocated %u bytes (%u frames). Use at most that.",
                 config->dma_buffer_size, (unsigned)actual,
                 (unsigned)(actual / (size_t)bytes_per_frame));
        ret = ESP_ERR_INVALID_SIZE;
        goto fail;
    }

    ret = i2s_channel_register_event_callback(rx, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_register_event_callback failed: %s", esp_err_to_name(ret));
        goto fail;
    }

    /* One wait slice = two buffer periods, so a stopped reader returns
     * promptly but a running one rarely wakes without data. */
    period_ms = (uint32_t)(((uint64_t)frames * 1000 + (uint64_t)config->sample_rate - 1) /
                                    (uint64_t)config->sample_rate);
    slice_ms = 2 * period_ms;
    if (slice_ms < I2S_MIC_MIN_SLICE_MS) {
        slice_ms = I2S_MIC_MIN_SLICE_MS;
    }

    s_mic.rx = rx;
    s_mic.buf_size = actual;
    s_mic.slice_ms = slice_ms;
    s_mic.overflow_count = 0;
    s_mic.running = false;
    s_mic.reader_active = false;

    portENTER_CRITICAL(&s_mic.mux);
    s_mic.state = MIC_STATE_INITIALIZED;
    portEXIT_CRITICAL(&s_mic.mux);
    return ESP_OK;

fail:
    i2s_del_channel(rx);
    return ret;
}

esp_err_t i2s_mic_deinit(void)
{
    portENTER_CRITICAL(&s_mic.mux);
    if (s_mic.state != MIC_STATE_INITIALIZED || s_mic.reader_active) {
        portEXIT_CRITICAL(&s_mic.mux);
        return ESP_ERR_INVALID_STATE;
    }
    /* From here no new read can start: i2s_mic_read() checks the state
     * under the same lock before claiming the reader slot. */
    s_mic.state = MIC_STATE_UNINITIALIZED;
    portEXIT_CRITICAL(&s_mic.mux);

    esp_err_t ret = i2s_del_channel(s_mic.rx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_del_channel failed: %s", esp_err_to_name(ret));
    }
    s_mic.rx = NULL;
    s_mic.buf_size = 0;
    s_mic.slice_ms = 0;
    return ret;
}

esp_err_t i2s_mic_start(void)
{
    if (s_mic.state != MIC_STATE_INITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }

    /* The channel is disabled here, so the ISR can't touch the counter. */
    s_mic.overflow_count = 0;

    /* enable() also empties the driver's queue: no stale audio. */
    esp_err_t ret = i2s_channel_enable(s_mic.rx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }

    s_mic.running = true;
    s_mic.state = MIC_STATE_RUNNING;
    return ESP_OK;
}

esp_err_t i2s_mic_stop(void)
{
    if (s_mic.state != MIC_STATE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Clear first: any reader between wait slices now returns
     * ESP_ERR_INVALID_STATE instead of starting a new driver wait. */
    s_mic.running = false;

    /* Waits for a driver copy already in progress (at most about one buffer
     * period plus the reader's scheduling delay), then stops the DMA. */
    esp_err_t ret = i2s_channel_disable(s_mic.rx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_disable failed: %s", esp_err_to_name(ret));
    }

    s_mic.state = MIC_STATE_INITIALIZED;
    return ret;
}

/* ------------------------------------------------------------------------
 * Read
 * ---------------------------------------------------------------------- */

static esp_err_t read_one_buffer(void *dst, size_t len, uint32_t timeout_ms)
{
    const bool forever = (timeout_ms == (uint32_t)portMAX_DELAY);
    const TickType_t t0 = xTaskGetTickCount();

    for (;;) {
        if (!s_mic.running) {
            return ESP_ERR_INVALID_STATE;
        }

        uint32_t wait_ms = s_mic.slice_ms;
        bool last_slice = false;
        if (!forever) {
            uint32_t elapsed_ms = (uint32_t)(xTaskGetTickCount() - t0) * portTICK_PERIOD_MS;
            uint32_t remaining_ms = (elapsed_ms >= timeout_ms) ? 0 : timeout_ms - elapsed_ms;
            if (remaining_ms <= wait_ms) {
                wait_ms = remaining_ms;
                last_slice = true;
            }
        }

        size_t n = 0;
        esp_err_t r = i2s_channel_read(s_mic.rx, dst, len, &n, wait_ms);

        if (r == ESP_OK) {
            /* A short ESP_OK (including 0 bytes) means the channel was
             * stopped during the read. */
            return (n == len) ? ESP_OK : ESP_ERR_INVALID_STATE;
        }
        if (r != ESP_ERR_TIMEOUT && r != ESP_ERR_INVALID_STATE) {
            return r;
        }

        /* Timed-out or not-yet-enabled slice: nothing was consumed. */
        if (!s_mic.running) {
            return ESP_ERR_INVALID_STATE;
        }
        if (last_slice) {
            return ESP_ERR_TIMEOUT;
        }
    }
}

esp_err_t i2s_mic_read(void *dst, size_t len, size_t *bytes_read, uint32_t timeout_ms)
{
    if (bytes_read) {
        *bytes_read = 0;
    }
    if (dst == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Check the state and claim the single reader slot atomically, so
     * deinit() can't delete the channel under a starting read. */
    portENTER_CRITICAL(&s_mic.mux);
    if (s_mic.state == MIC_STATE_UNINITIALIZED || s_mic.reader_active) {
        portEXIT_CRITICAL(&s_mic.mux);
        return ESP_ERR_INVALID_STATE;
    }
    if (len != s_mic.buf_size) {
        portEXIT_CRITICAL(&s_mic.mux);
        return ESP_ERR_INVALID_SIZE;
    }
    s_mic.reader_active = true;
    portEXIT_CRITICAL(&s_mic.mux);

    esp_err_t ret = read_one_buffer(dst, len, timeout_ms);

    portENTER_CRITICAL(&s_mic.mux);
    s_mic.reader_active = false;
    portEXIT_CRITICAL(&s_mic.mux);

    if (ret == ESP_OK && bytes_read) {
        *bytes_read = len;
    }
    return ret;
}

/* ------------------------------------------------------------------------
 * Queries
 * ---------------------------------------------------------------------- */

uint32_t i2s_mic_get_overflow_count(void)
{
    return s_mic.overflow_count;
}

size_t i2s_mic_get_buffer_size(void)
{
    return (s_mic.state == MIC_STATE_UNINITIALIZED) ? 0 : s_mic.buf_size;
}
