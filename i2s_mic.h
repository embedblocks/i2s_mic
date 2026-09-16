/*
 * i2s_mic — ISR-driven, task-free I2S microphone component for ESP-IDF v6.0.x
 *
 * See the design document ("ESP-IDF I2S Microphone and Speaker Components")
 * and its companion implementation specification for the full contract this
 * header implements. In particular:
 *
 *   - The component has no internal FreeRTOS task. `mic_buffer_ready_cb_t`
 *     and `mic_overflow_cb_t` run in ISR context, invoked inline from the
 *     I2S driver's DMA-completion dispatch. Read Section 4 of the design
 *     document before writing either callback.
 *   - `i2s_mic_request_buffer()` is safe to call from an ordinary task or
 *     from inside `mic_buffer_ready_cb_t` itself.
 *   - This is a singleton component: only one `i2s_mic` instance exists per
 *     firmware image (see the design document's Open Question 5).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Invoked (in ISR context) when a previously posted application
 *        buffer has just been filled with newly captured audio.
 *
 * The callback MUST follow the ISR-context rules in the design document's
 * Section 4.1: no blocking FreeRTOS calls without the FromISR suffix, no
 * heap allocation, no I/O, no logging, no mutexes, and it must return as
 * quickly as possible. The normal pattern is to hand the buffer pointer to
 * an application task (queue/notify) and, if desired, call
 * i2s_mic_request_buffer() again to re-post a buffer from a pre-allocated
 * pool.
 *
 * @param buffer     The application-owned buffer that was filled. This is
 *                    the same pointer the application passed to
 *                    i2s_mic_request_buffer().
 * @param bytes_read  Number of bytes actually written into `buffer`.
 * @param user_ctx    The `user_ctx` pointer from i2s_mic_config_t.
 */
typedef void (*mic_buffer_ready_cb_t)(uint8_t *buffer, size_t bytes_read, void *user_ctx);

/**
 * @brief Invoked (in ISR context) whenever either RX drop condition occurs:
 *        ESP-IDF's own internal DMA queue overflow, or no application
 *        buffer having been posted at DMA-completion time.
 *
 * Fires once per newly observed drop event (never batched). Both cumulative
 * counters are always passed, regardless of which condition triggered this
 * particular invocation. Both counters reset to zero on every successful
 * i2s_mic_start().
 *
 * @param total_overflow_count   Cumulative count of ESP-IDF's own
 *                                on_recv_q_ovf events since the last start().
 * @param total_no_buffer_count  Cumulative count of on_recv events for which
 *                                no application buffer was posted, since the
 *                                last start().
 * @param user_ctx                The `user_ctx` pointer from i2s_mic_config_t.
 */
typedef void (*mic_overflow_cb_t)(uint32_t total_overflow_count,
                                   uint32_t total_no_buffer_count,
                                   void *user_ctx);

typedef struct {
    int sample_rate;

    /** 16 or 32 only. 24-bit is not supported in V1 (see implementation spec Section 7). */
    int bits_per_sample;

    /** 1 = mono, 2 = stereo. Must match slot_mode exactly (validated at init()). */
    int channel_count;

    /** I2S_SLOT_MODE_MONO or I2S_SLOT_MODE_STEREO; must match channel_count. */
    i2s_slot_mode_t slot_mode;

    /** Mandatory GPIOs — there is no "unused" convention for these in V1. */
    int gpio_bck;
    int gpio_ws;
    int gpio_data;

    /** I2S peripheral/port to use (e.g. I2S_NUM_0, I2S_NUM_1). Plain `int`
     *  because ESP-IDF v6.0 removed the `i2s_port_t` type in favor of int.
     *  See the implementation spec Section 10 for guidance on port
     *  selection when running i2s_mic and i2s_spk simultaneously. */
    int port;

    /** Passed directly as ESP-IDF's own DMA descriptor count (dma_desc_num).
     *  Not independently validated by this component. */
    int dma_buffer_count;

    /** Bytes of audio payload delivered per DMA completion / per posted
     *  application buffer. Must be an exact multiple of
     *  channel_count * (bits_per_sample / 8), or init() fails with
     *  ESP_ERR_INVALID_ARG. */
    int dma_buffer_size;

    /** Max buffers that may be posted and not yet delivered at once.
     *  Valid range: [1, 16]. */
    int max_pending_buffers;

    /** Mandatory. init() fails with ESP_ERR_INVALID_ARG if NULL. */
    mic_buffer_ready_cb_t cb;

    /** Optional. NULL is accepted; drop conditions are simply not reported. */
    mic_overflow_cb_t overflow_cb;

    void *user_ctx;
} i2s_mic_config_t;

/**
 * @brief Initialize the microphone component.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_ARG    NULL config; NULL cb; unsupported
 *                            bits_per_sample (not 16 or 32);
 *                            channel_count/slot_mode mismatch;
 *                            max_pending_buffers outside [1,16];
 *                            dma_buffer_size not an exact multiple of
 *                            channel_count * (bits_per_sample/8)
 *   - ESP_ERR_INVALID_STATE  already INITIALIZED or RUNNING
 *   - ESP_ERR_NO_MEM         allocation failure
 *   - any other esp_err_t propagated unchanged from the underlying
 *     ESP-IDF I2S driver calls (i2s_new_channel, i2s_channel_init_std_mode,
 *     i2s_channel_register_event_callback)
 */
esp_err_t i2s_mic_init(i2s_mic_config_t *config);

/**
 * @brief Tear down the microphone component and release the underlying
 *        ESP-IDF I2S channel.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not INITIALIZED — must stop() first
 *         if RUNNING), or an esp_err_t propagated unchanged from
 *         i2s_del_channel().
 */
esp_err_t i2s_mic_deinit(void);

/**
 * @brief Start capturing. Resets both overflow counters to zero.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not INITIALIZED), or an esp_err_t
 *         propagated unchanged from i2s_channel_enable().
 */
esp_err_t i2s_mic_start(void);

/**
 * @brief Stop capturing.
 *
 * Blocks until every in-flight mic_buffer_ready_cb_t / mic_overflow_cb_t
 * dispatch has returned, so that no such callback fires after this function
 * returns. Any buffers posted but not yet filled are discarded (their
 * contents become undefined; no callback fires for them). This wait is
 * unbounded — if the application's callback misbehaves and blocks, stop()
 * waits correspondingly long. See the implementation specification's
 * Section 5 for the full guarantee.
 *
 * @return ESP_OK, ESP_ERR_INVALID_STATE (not RUNNING), or an esp_err_t
 *         propagated unchanged from i2s_channel_disable().
 */
esp_err_t i2s_mic_stop(void);

/**
 * @brief Post an application-owned buffer to be filled with captured audio
 *        at a future DMA completion.
 *
 * Non-blocking; returns immediately. Safe to call from an ordinary
 * application task or from within mic_buffer_ready_cb_t itself (ISR
 * context) — both call paths share the same synchronization.
 *
 * @param buffer      Application-owned buffer, at least dma_buffer_size
 *                     bytes (from i2s_mic_config_t). Must not currently be
 *                     pending (already posted but not yet delivered).
 * @param buffer_len   Length of `buffer` in bytes.
 *
 * @return
 *   - ESP_OK
 *   - ESP_ERR_INVALID_ARG    buffer is NULL, buffer_len is 0, or this exact
 *                            pointer is already pending
 *   - ESP_ERR_INVALID_SIZE   buffer_len < dma_buffer_size
 *   - ESP_ERR_NO_MEM         max_pending_buffers are already outstanding
 *   - ESP_ERR_INVALID_STATE  component is UNINITIALIZED, or i2s_mic_stop()
 *                            is currently in progress
 */
esp_err_t i2s_mic_request_buffer(uint8_t *buffer, size_t buffer_len);

#ifdef __cplusplus
}
#endif
