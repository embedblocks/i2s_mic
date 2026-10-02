/*
 * i2s_mic — thin, transparent I2S microphone component for ESP-IDF v6.0.x
 *
 * The component configures an I2S RX channel (standard Philips mode,
 * master role) and exposes a blocking read. It adds no buffering of its
 * own: audio waits in the driver's DMA buffers until the application calls
 * i2s_mic_read() from its own task. The application decides when to copy,
 * copies straight into its own destination, and owns the consequences of
 * being late, which i2s_mic_get_overflow_count() reports honestly.
 *
 * Contract summary (details on each function below):
 *
 *   - Every i2s_mic_read() call reads exactly one DMA buffer
 *     (len == i2s_mic_get_buffer_size()). Each call either delivers a full
 *     buffer or delivers nothing.
 *   - One reading task only. A second concurrent reader is rejected.
 *   - Never suspend or delete the reading task while it is inside
 *     i2s_mic_read().
 *   - Lifecycle calls (init/start/stop/deinit) must not race each other.
 *     Call them from one control task. stop() may be called while another
 *     task is blocked in i2s_mic_read(); that is the intended shutdown path.
 *   - Shutdown order: i2s_mic_stop() -> wait for the reading task to see
 *     ESP_ERR_INVALID_STATE and leave its loop -> i2s_mic_deinit().
 *   - Buffers are not guaranteed untorn. A reader that falls a full DMA
 *     ring behind and is then preempted mid-copy can receive a buffer the
 *     DMA has started rewriting. When that is possible, the overflow count
 *     is already rising, so treat any rise as "audio unreliable".
 *
 * This is a singleton: one i2s_mic instance per firmware image.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2s_std.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** Sample rate in Hz, e.g. 16000. */
    int sample_rate;

    /** 16 or 32 only. 24-bit mics (INMP441, MSM261S4030H0, ...) deliver
     *  their samples MSB-justified in a 32-bit slot: use 32. */
    int bits_per_sample;

    /** 1 = mono, 2 = stereo. Must match slot_mode. */
    int channel_count;

    /** I2S_SLOT_MODE_MONO or I2S_SLOT_MODE_STEREO; must match channel_count. */
    i2s_slot_mode_t slot_mode;

    /** Bit clock, word select and data GPIOs. All mandatory. */
    int gpio_bck;
    int gpio_ws;
    int gpio_data;

    /** I2S port, e.g. I2S_NUM_0. Plain int: ESP-IDF v6.0 has no i2s_port_t. */
    int port;

    /** Number of DMA buffers in the driver's ring (dma_desc_num), >= 2.
     *  The driver can hold dma_buffer_count - 1 finished buffers for the
     *  reader; that is how far behind the reader may fall before audio is
     *  dropped. 6 is a good default. */
    int dma_buffer_count;

    /** Bytes per DMA buffer. This is also the exact length every
     *  i2s_mic_read() call must request. Must be a multiple of
     *  channel_count * (bits_per_sample / 8), and must fit in one DMA
     *  buffer (4092 bytes on most targets). init() rejects a size the
     *  driver would silently shrink. */
    int dma_buffer_size;
} i2s_mic_config_t;

/**
 * @brief Create and configure the I2S RX channel. Does not start capture.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_ARG    NULL config; bits_per_sample not 16 or 32;
 *                            channel_count / slot_mode mismatch;
 *                            sample_rate <= 0; dma_buffer_count < 2;
 *                            dma_buffer_size <= 0 or not a whole number of
 *                            frames
 *   - ESP_ERR_INVALID_SIZE   dma_buffer_size is larger than the driver can
 *                            allocate as one DMA buffer
 *   - ESP_ERR_INVALID_STATE  already initialized
 *   - any other esp_err_t from i2s_new_channel(),
 *     i2s_channel_init_std_mode(), i2s_channel_register_event_callback()
 */
esp_err_t i2s_mic_init(const i2s_mic_config_t *config);

/**
 * @brief Release the I2S channel. Call after i2s_mic_stop(), once the
 *        reading task has left i2s_mic_read().
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_STATE  not initialized, still running, or a task is
 *                            still inside i2s_mic_read() (retry after it
 *                            returns)
 *   - any other esp_err_t from i2s_del_channel()
 */
esp_err_t i2s_mic_deinit(void);

/**
 * @brief Start capturing. Discards any audio left in the driver from a
 *        previous run and resets the overflow count to 0.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not initialized, or already
 *         running), or an esp_err_t from i2s_channel_enable().
 */
esp_err_t i2s_mic_start(void);

/**
 * @brief Stop capturing.
 *
 * A task blocked in i2s_mic_read() wakes up and returns either one last
 * full buffer (ESP_OK) or ESP_ERR_INVALID_STATE. stop() itself may block
 * for up to about one DMA buffer period plus the reading task's scheduling
 * delay, because the driver waits for an in-progress copy to finish.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not running), or an esp_err_t
 *         from i2s_channel_disable(). The component is stopped in every
 *         case except ESP_ERR_INVALID_STATE.
 */
esp_err_t i2s_mic_stop(void);

/**
 * @brief Wait for the next DMA buffer and copy it into dst.
 *
 * Call from one task only. Blocks until a buffer is ready, the timeout
 * expires, or i2s_mic_stop() is called.
 *
 * @param dst         Destination, at least len bytes.
 * @param len         Must equal i2s_mic_get_buffer_size().
 * @param bytes_read  Out, optional (may be NULL): len on ESP_OK, 0 on any
 *                    error.
 * @param timeout_ms  Maximum wait in ms. portMAX_DELAY waits until a buffer
 *                    arrives or capture stops. 0 returns at once if no
 *                    buffer is ready.
 *
 * @return
 *   - ESP_OK                 one full buffer copied into dst
 *   - ESP_ERR_TIMEOUT        no buffer arrived within timeout_ms
 *   - ESP_ERR_INVALID_STATE  not running (returned immediately), stopped
 *                            while waiting, or another task is already
 *                            inside i2s_mic_read()
 *   - ESP_ERR_INVALID_SIZE   len != i2s_mic_get_buffer_size()
 *   - ESP_ERR_INVALID_ARG    dst is NULL
 *   - any other esp_err_t from i2s_channel_read()
 */
esp_err_t i2s_mic_read(void *dst, size_t len, size_t *bytes_read, uint32_t timeout_ms);

/**
 * @brief Number of DMA buffers the driver discarded because the reader was
 *        too far behind, since the last i2s_mic_start().
 *
 * Each count is one lost buffer of audio (dma_buffer_size bytes). 0 while
 * the reader keeps up. Safe to call from any task.
 */
uint32_t i2s_mic_get_overflow_count(void);

/**
 * @brief Bytes per DMA buffer, which is the exact length i2s_mic_read()
 *        requires. 0 when not initialized.
 */
size_t i2s_mic_get_buffer_size(void);

#ifdef __cplusplus
}
#endif
