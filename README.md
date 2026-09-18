# i2s_mic

![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.x-blue)
![Espressif Component Registry](https://img.shields.io/badge/Espressif-Component%20Registry-orange)
![License](https://img.shields.io/badge/license-MIT-green)

Task-free, ISR-driven I2S microphone component for ESP-IDF, built on the
modern `i2s_std` driver. Post application-owned buffers, get them back
filled with captured audio via a callback — no internal FreeRTOS task, no
DMA-owned pointers ever handed to your code, no hidden copies beyond the
one from DMA into your buffer.

Works with any I2S digital MEMS microphone that speaks the standard
Philips slot format (INMP441, MSM261S4030H0, and similar parts using the
same 24-bit-in-32-bit layout — see the compatibility note under Notes).

---

## Features

* **No internal FreeRTOS task** — `mic_buffer_ready_cb_t` and
  `mic_overflow_cb_t` run inline from the I2S driver's own ISR dispatch;
  no extra task, no extra stack, no scheduling latency of its own
* **Application-owned buffers** — you pre-allocate and post buffers with
  `i2s_mic_request_buffer()`; the component copies captured audio into the
  oldest posted buffer on each DMA completion and hands it back via
  callback. It never hands you a DMA-owned pointer
* **Dual overflow tracking** — ESP-IDF's own internal DMA queue overflow
  and "no application buffer was posted in time" are tracked and reported
  separately through `mic_overflow_cb_t`, so you know which failure mode
  you're actually hitting
* **Dual-context-safe posting** — `i2s_mic_request_buffer()` is safe to
  call from an ordinary task or from inside the ISR callback itself, using
  the same synchronization either way
* **Zero heap allocation on the hot path** — buffers are entirely
  caller-supplied; the only allocation is a small internal bookkeeping
  array sized once at `init()`

---

## Chip Support

| Chip | Status |
|---|---|
| ESP32 | Confirmed |
| ESP32-C3 | Confirmed |
| ESP32-S3 | Expected to work (`SOC_I2S_NUM >= 1`) |
| ESP32-C6 | Expected to work (`SOC_I2S_NUM >= 1`) |

Any target with `SOC_I2S_NUM >= 1` should work via the standard driver;
only the two listed above have been directly tested against this
component's specific implementation.

---

## Installation

```bash
idf.py add-dependency "embedblocks/i2s_mic^0.1.3"
```

Or in `idf_component.yml`:

```yaml
dependencies:
  embedblocks/i2s_mic: "^0.1.3"
```

---

## Supported Formats (V1)

- `bits_per_sample`: 16 or 32 only (24-bit is rejected — see "Implementation
  notes" below for why).
- Philips I2S standard slot format only (no MSB-justified/PCM/TDM, no MCLK,
  no signal inversion).
- `I2S_ROLE_MASTER` only.
- `max_pending_buffers` in `[1, 16]`.

---

## Usage

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
        .channel_count = 1,        // see "Mono RX caveat" below if
        .slot_mode = I2S_SLOT_MODE_MONO,  // this doesn't behave as expected
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

---

## Examples

| Example | Transport | Board type | Notes |
|---|---|---|---|
| `examples/uart-jtag` | Native USB-Serial-JTAG | ESP32-C3/S3/C6 boards with no separate UART bridge chip | Single cable, no extra hardware |
| `examples/uart` | UART0 (shared with console) | Classic ESP32 boards with a CP2102/CH340-style bridge chip | Single cable, no extra hardware |

Both examples capture from an I2S mic and stream 16-bit PCM to a PC script
that plays it back live or records it to a WAV file.

---

## Mono RX Caveat: Read This Before You Wire Up Mono

On at least one tested ESP32-C3 / ESP-IDF v6.0.x combination, configuring
`channel_count = 1` / `I2S_SLOT_MODE_MONO` did **not** produce the compact,
single-slot-per-frame buffer ESP-IDF's own documentation describes for
ESP32/S2 RX. Instead, the DMA buffer this component copied from still
contained *both* I2S slots interleaved — every other 32-bit word was real
microphone audio, and the words in between carried something else (in the
diagnosed case, a spectrum dominated by ~50 Hz mains hum rather than
audio, consistent with a floating/unused slot). `i2s_mic` faithfully copies
whatever the DMA-completion event reports; it has no way to know the
underlying slot content doesn't match what `channel_count = 1` implies.

This was diagnosed empirically from one board/IDF build via signal
analysis of a captured recording (near-zero sample-to-sample
autocorrelation alternating with strong correlation two samples apart —
the signature of two interleaved but unrelated streams), not from an
ESP-IDF changelog or errata entry, so it isn't confirmed to be universal
across all ESP32-C3 units or all v6.0.x point releases. If your mono
recording sounds badly distorted (not just quiet or noisy, but a
heavily distorted, low-pitched "growl" at otherwise-correct length):

1. Reconfigure `i2s_mic_init()` for `channel_count = 2` /
   `I2S_SLOT_MODE_STEREO` instead of mono, and double `dma_buffer_size`
   accordingly (the byte-accounting formula in `i2s_mic_config_t` already
   handles this: `dma_buffer_size` must be an exact multiple of
   `channel_count * (bits_per_sample / 8)`).
2. In your `mic_buffer_ready_cb_t`, keep only every other 32-bit sample
   (either the even-indexed or odd-indexed word of each pair — check both,
   the real signal should look like plausible audio and the discarded one
   should look like near-silence or narrowband noise) instead of treating
   every word as audio.

Both example projects implement this workaround already (`MIC_CHANNEL_COUNT`,
`KEEP_SLOT` in their `main/app_main.c`).

---

## Notes

**Singleton.** One `i2s_mic` instance per firmware image — calling
`init()` a second time before `deinit()` returns `ESP_ERR_INVALID_STATE`.

**Microphone compatibility is broader than the name suggests.** `i2s_mic`
has no part-specific logic at all — it's a generic I2S receiver. It was
developed and directly tested with an **MSM261S4030H0**, which worked
unmodified. INMP441 is used throughout this README and the examples as
the reference part in wiring diagrams — it's the most widely documented
I2S MEMS mic — but it has **not** actually been tested against this
component; compatibility with it is expected, not confirmed, based on it
sharing the same 24-bit-in-32-bit Philips format as the MSM261S4030H0.
Other parts using that same format (e.g. ICS-43434, SPH0645) are likely
compatible for the same reason, also untested. If you try the INMP441 (or
any other part) and confirm it works, that's useful information worth
updating this note with.


**ISR callback rules are ordinary ISR rules.** `mic_buffer_ready_cb_t` and
`mic_overflow_cb_t` must obey the same constraints as any ISR: no blocking
FreeRTOS calls without the `FromISR` suffix, no heap allocation, no I/O,
no logging, no mutexes, return quickly.

**`i2s_mic_request_buffer()` is dual-context-safe by design** — call it
from a task or from inside the callback itself, both are supported with
the same synchronization underneath.

---

## Implementation Notes / Judgment Calls

The design and implementation-specification documents this component was
built from are unusually complete, but a few points were left open and
required a decision during implementation. Each is called out in code
comments at the relevant spot too:

1. **`i2s_port_t` doesn't exist in ESP-IDF v6.0, and `REQUIRES driver` no
   longer pulls in UART/I2S headers.** Since v5.3, ESP-IDF has been
   splitting the monolithic `driver` component into per-peripheral
   components (`esp_driver_uart`, `esp_driver_i2s`, `esp_driver_gpio`,
   etc.); in v6.0, depending on just `driver` is no longer sufficient to
   get `driver/uart.h` or `driver/i2s_std.h`. This component's
   `CMakeLists.txt` requires `esp_driver_i2s` explicitly. Alongside that,
   the legacy I2S driver was removed entirely in v6.0, taking the
   `i2s_port_t` type with it — ESP-IDF now uses plain `int` for port
   numbers (`I2S_NUM_0`, `I2S_NUM_1`, etc. are still defined as int-valued
   constants). `i2s_mic_config_t::port` is typed `int` accordingly.
   Similarly, `i2s_event_data_t`'s buffer pointer field was renamed from
   `data` to `dma_buf` — this component uses `dma_buf`, which does **not**
   exist prior to that rename. Building against an older ESP-IDF release
   would require reverting all of the above (the `REQUIRES`, the `port`
   type, and `dma_buf`) to their pre-v6.0 equivalents.

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
   forward declaration — doing it in both places makes GCC treat them as
   two different IRAM placement requests for the same symbol, which fails
   under `-Werror=attributes`.)

Everything else — the state machine, the two-lock model, the pending-buffer
queue's FIFO/duplicate-detection/ISR-safety requirements, the overflow
counter semantics, and the byte-accounting formula for `dma_buffer_size` —
is implemented exactly as specified, with no invented behavior beyond
those three points.

---

## Known Limitations

- Multi-instance / handle-based API is out of scope for V1 (singleton
  only).
- MSB-justified/PCM/TDM slot formats, MCLK, and signal inversion are not
  supported.
- 24-bit audio is not supported (`bits_per_sample` must be 16 or 32).
- No coordination with a companion speaker component (e.g. automatic
  port-conflict detection on single-I2S-peripheral chips) — the
  application is responsible for correct port assignment.
- No "no-copy" posting mode — every DMA completion copies into your
  posted buffer once.

---

## Requirements

- ESP-IDF v6.0.x (v6.0–v6.0.3 verified against the pinned header behavior
  this component relies on; re-verify `i2s_channel_write`/`disable`/`enable`
  semantics against any later v6.0.x patch before upgrading).
- A target with `SOC_I2S_NUM >= 1` (see Chip Support above).

---

## License

MIT License — see LICENSE file.
