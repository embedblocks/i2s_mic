# i2s_mic

Task-free, ISR-driven I2S microphone component for ESP-IDF **v6.0.x**, built
on the modern `i2s_std` driver (`driver/i2s_std.h`). Implements the contract
described in the accompanying design document and implementation
specification.

## Key properties

- **No internal FreeRTOS task.** `mic_buffer_ready_cb_t` and
  `mic_overflow_cb_t` run in ISR context, invoked inline from the I2S
  driver's DMA-completion dispatch. Your callback must obey ordinary ISR
  rules: no blocking FreeRTOS calls without the `FromISR` suffix, no heap
  allocation, no I/O, no logging, no mutexes, return quickly.
- **Singleton.** One `i2s_mic` instance per firmware image (`init()` a
  second time before `deinit()` returns `ESP_ERR_INVALID_STATE`).
- **Application-owned buffers.** You pre-allocate buffers and post them with
  `i2s_mic_request_buffer()`; the component copies captured audio into the
  oldest posted buffer on each DMA completion and hands it back via
  `mic_buffer_ready_cb_t`. It never hands you a DMA-owned pointer.
- **Two independently-tracked drop conditions**, both reported through
  `mic_overflow_cb_t`: ESP-IDF's own internal DMA queue overflow, and "no
  application buffer was posted in time."

## Supported formats (V1)

- `bits_per_sample`: 16 or 32 only (24-bit is rejected — see the
  implementation spec's Section 7 for why).
- Philips I2S standard slot format only (no MSB-justified/PCM/TDM, no MCLK,
  no signal inversion).
- `I2S_ROLE_MASTER` only.
- `max_pending_buffers` in `[1, 16]`.

## Minimal usage

```c
#include "i2s_mic.h"

#define NUM_BUFS   3
#define BUF_BYTES  1024   // must equal dma_buffer_size below

static uint8_t s_bufs[NUM_BUFS][BUF_BYTES];
static QueueHandle_t s_audio_q;

static void IRAM_ATTR mic_ready(uint8_t *buffer, size_t bytes_read, void *ctx)
{
    // ISR context: hand off, don't process.
    BaseType_t hp_task_woken = pdFALSE;
    xQueueSendFromISR(s_audio_q, &buffer, &hp_task_woken);

    // Re-post from the same pre-allocated pool (never a fresh allocation here).
    i2s_mic_request_buffer(buffer, BUF_BYTES);

    portYIELD_FROM_ISR(hp_task_woken);
}

static void mic_overflow(uint32_t total_overflow, uint32_t total_no_buffer, void *ctx)
{
    // ISR context — keep this as short as mic_ready.
}

void app_main(void)
{
    s_audio_q = xQueueCreate(NUM_BUFS, sizeof(uint8_t *));

    i2s_mic_config_t cfg = {
        .sample_rate = 16000,
        .bits_per_sample = 16,
        .channel_count = 1,
        .slot_mode = I2S_SLOT_MODE_MONO,
        .gpio_bck = GPIO_NUM_4,
        .gpio_ws = GPIO_NUM_5,
        .gpio_data = GPIO_NUM_6,
        .port = I2S_NUM_0,
        .dma_buffer_count = 6,
        .dma_buffer_size = BUF_BYTES,
        .max_pending_buffers = NUM_BUFS,
        .cb = mic_ready,
        .overflow_cb = mic_overflow,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(i2s_mic_init(&cfg));

    for (int i = 0; i < NUM_BUFS; i++) {
        ESP_ERROR_CHECK(i2s_mic_request_buffer(s_bufs[i], BUF_BYTES));
    }

    ESP_ERROR_CHECK(i2s_mic_start());

    uint8_t *filled;
    while (1) {
        if (xQueueReceive(s_audio_q, &filled, portMAX_DELAY) == pdTRUE) {
            // Ordinary task context: do the slow work here (UART, network, etc).
        }
    }
}
```

## Implementation notes / judgment calls

The design and implementation-specification documents are unusually
complete, but two points were left open and required a decision during
implementation. Both are called out in code comments at the relevant spot:

1. **`i2s_port_t` doesn't exist in ESP-IDF v6.0, and `REQUIRES driver` no
   longer pulls in UART/I2S headers.** Since v5.3, ESP-IDF has been
   splitting the monolithic `driver` component into per-peripheral
   components (`esp_driver_uart`, `esp_driver_i2s`, `esp_driver_gpio`,
   etc.); in v6.0, depending on just `driver` is no longer sufficient to
   get `driver/uart.h` or `driver/i2s_std.h`. This component's
   `CMakeLists.txt` now requires `esp_driver_i2s` explicitly, and the UART
   example's `main/CMakeLists.txt` requires `esp_driver_uart`. Alongside
   that, the legacy I2S driver was removed entirely in v6.0, taking the
   `i2s_port_t` type with it — ESP-IDF now uses plain `int` for port
   numbers (`I2S_NUM_0`, `I2S_NUM_1`, etc. are still defined as int-valued
   constants). `i2s_mic_config_t::port` is typed `int` accordingly.
   Similarly, `i2s_event_data_t`'s buffer pointer field was renamed from
   `data` to `dma_buf` — this component uses `dma_buf`, which does **not**
   exist prior to that rename. If you need to build against an older
   ESP-IDF release, all of the above (the `REQUIRES`, the `port` type, and
   `dma_buf`) would need to be reverted to their pre-v6.0 equivalents.

2. **`i2s_mic_stop()` when `i2s_channel_disable()` itself fails.** Neither
   document specifies whether to still complete the in-flight wait and
   queue-clear in this case. This implementation always completes them
   (so the component's internal bookkeeping — `is_running`, `stopping`,
   the pending queue, the public lifecycle state — stays consistent for
   future calls), while still propagating the original `esp_err_t` from
   `i2s_channel_disable()` to the caller unchanged, per the general
   error-propagation rule.

3. **`IRAM_ATTR` placement.** The design document requires the ISR copy
   path (and everything it calls, including `i2s_mic_request_buffer()`) to
   live in IRAM *when* `CONFIG_I2S_ISR_IRAM_SAFE` is enabled. This
   implementation tags `mic_on_recv`, `mic_on_recv_q_ovf`, and
   `i2s_mic_request_buffer()` with `IRAM_ATTR` unconditionally, which is
   correct in both configurations but permanently costs a small, fixed
   amount of IRAM even when the Kconfig option is off. If IRAM is tight in
   your project and you never enable `CONFIG_I2S_ISR_IRAM_SAFE`, these
   three `IRAM_ATTR` tags can be removed. (Implementation detail: the
  attribute is applied only at each function's *definition*, not on its
  forward declaration — doing it in both places makes GCC treat them as two
  different IRAM placement requests for the same symbol, which fails under
  `-Werror=attributes`.)

Everything else — the state machine, the two-lock model, the pending-buffer
queue's FIFO/duplicate-detection/ISR-safety requirements, the overflow
counter semantics, and the byte-accounting formula for `dma_buffer_size` —
is implemented exactly as specified, with no invented behavior beyond
those two points.

## Not implemented (explicitly out of scope for V1, per spec)

- Multi-instance / handle-based API (singleton only).
- MSB-justified/PCM/TDM slot formats, MCLK, signal inversion.
- 24-bit audio.
- Coordination with `i2s_spk` (e.g., automatic port-conflict detection on
  single-I2S-peripheral chips) — the application is responsible for correct
  port assignment; see the implementation spec's Section 10.
- A "no-copy" posting mode.

## Requirements

- ESP-IDF v6.0.x (v6.0–v6.0.3 verified against the pinned header behavior
  this component relies on; re-verify `i2s_channel_write`/`disable`/`enable`
  semantics against any later v6.0.x patch before upgrading).
- Targets with `SOC_I2S_NUM >= 1` (ESP32, ESP32-S3, ESP32-C3, ESP32-C6
  confirmed; other targets should work via the standard driver but are
  unverified against this spec).
