# i2s_mic

![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v6.0.x-blue)
![Espressif Component Registry](https://img.shields.io/badge/Espressif-Component%20Registry-orange)
![License](https://img.shields.io/badge/license-MIT-green)

A thin, transparent I2S microphone component for ESP-IDF, built on the
modern `i2s_std` driver. It sets up the RX channel and gives you a
blocking `i2s_mic_read()`. Your task reads one DMA buffer at a time, into
your own memory, on your own schedule. If your task falls too far behind,
the driver drops audio and `i2s_mic_get_overflow_count()` tells you how
much.

Works with any I2S digital MEMS microphone that speaks the standard
Philips slot format (INMP441, MSM261S4030H0, and similar parts using the
same 24-bit-in-32-bit layout — see Notes).

---

## Features

* **Read when you're ready** — `i2s_mic_read()` blocks until the next DMA
  buffer is finished, then copies it into your buffer. No callbacks, no
  ISR rules for your code: log, block and process normally.
* **No hidden buffering** — finished buffers wait in the I2S driver's own
  queue (`dma_buffer_count - 1` of them) until you read them. i2s_mic adds
  no queue, no task and no locks of its own.
* **One honest loss counter** — `i2s_mic_get_overflow_count()` counts DMA
  buffers the driver discarded because your reader was too far behind.
  It stays at 0 while you keep up.
* **All-or-nothing reads** — each call returns a full buffer or nothing,
  so no audio is ever half-delivered.
* **Clean shutdown** — `i2s_mic_stop()` wakes a blocked reader within
  about one buffer period, whatever timeout it passed.
* **Checked DMA sizing** — `init()` rejects a buffer size the driver would
  silently shrink, so the size you configure is the size you get.

---

## Chip Support

| Chip | Status |
|---|---|
| ESP32 | Tested with 0.1.x |
| ESP32-C3 | Tested with 0.1.x |
| ESP32-S3 | Expected to work (`SOC_I2S_NUM >= 1`) |
| ESP32-C6 | Expected to work (`SOC_I2S_NUM >= 1`) |

Any target with `SOC_I2S_NUM >= 1` should work via the standard driver;
only the two listed above have been run on hardware, with the 0.1.x
implementation. 0.2.0 uses only public driver calls
(`i2s_channel_read()`, enable/disable, `on_recv_q_ovf`) and has not yet
been through the hardware acceptance tests.

---

## Installation

```bash
idf.py add-dependency "embedblocks/i2s_mic^0.2.0"
```

Or in `idf_component.yml`:

```yaml
dependencies:
  embedblocks/i2s_mic: "^0.2.0"
```

---

## Supported Formats

- `bits_per_sample`: 16 or 32 only (24-bit mics deliver their samples in a
  32-bit slot: use 32).
- Philips I2S standard slot format only (no MSB-justified/PCM/TDM, no MCLK,
  no signal inversion).
- `I2S_ROLE_MASTER` only.
- `dma_buffer_size` must fit in one DMA buffer: at most 4092 bytes on most
  targets (for example 511 stereo 32-bit frames, or 2046 mono 16-bit
  frames).

---

## Usage

```c
#include "i2s_mic.h"

#define BUF_FRAMES  480                        // 30 ms at 16 kHz
#define BUF_BYTES   (BUF_FRAMES * 2)           // mono, 16-bit

static int16_t s_buf[BUF_FRAMES];

static void audio_task(void *arg)
{
    size_t n;
    while (1) {
        esp_err_t ret = i2s_mic_read(s_buf, BUF_BYTES, &n, portMAX_DELAY);
        if (ret == ESP_ERR_INVALID_STATE) {
            break;                             // capture was stopped
        }
        if (ret != ESP_OK) {
            continue;
        }
        // Ordinary task context: process, log, send over UART/network...
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    i2s_mic_config_t cfg = {
        .sample_rate = 16000,
        .bits_per_sample = 16,
        .channel_count = 1,               // see "Mono RX caveat" below if
        .slot_mode = I2S_SLOT_MODE_MONO,  // this doesn't behave as expected
        .gpio_bck = GPIO_NUM_4,
        .gpio_ws = GPIO_NUM_5,
        .gpio_data = GPIO_NUM_6,
        .port = I2S_NUM_0,
        .dma_buffer_count = 6,
        .dma_buffer_size = BUF_BYTES,
    };

    ESP_ERROR_CHECK(i2s_mic_init(&cfg));
    ESP_ERROR_CHECK(i2s_mic_start());
    xTaskCreate(audio_task, "audio", 4096, NULL, 5, NULL);
}
```

### Read contract

Every call reads exactly one DMA buffer: `len` must equal
`i2s_mic_get_buffer_size()`.

| Situation | Returns | `bytes_read` |
|---|---|---|
| Buffer read | `ESP_OK` | full buffer |
| No buffer within `timeout_ms` | `ESP_ERR_TIMEOUT` | 0 |
| Called while not running | `ESP_ERR_INVALID_STATE`, immediately | 0 |
| `stop()` while waiting | `ESP_ERR_INVALID_STATE`, or `ESP_OK` with one last full buffer | 0 or full |
| Another task already inside `i2s_mic_read()` | `ESP_ERR_INVALID_STATE` | 0 |
| Wrong `len` | `ESP_ERR_INVALID_SIZE` | 0 |

### Rules

- **One reading task.** A second concurrent reader is rejected.
- **Never suspend or delete the reading task while it is inside
  `i2s_mic_read()`.** `i2s_mic_stop()` waits for the driver's in-progress
  copy, so a frozen reader would freeze `stop()`.
- **Give the reading task a priority above your processing**, so it keeps
  up even when processing is busy.
- **Call `init`/`start`/`stop`/`deinit` from one control task.** `stop()`
  may run while another task is blocked in `i2s_mic_read()`; that is the
  normal way to end capture.
- **Treat any rise in the overflow count as "audio unreliable".**

### Shutdown order

1. Call `i2s_mic_stop()`. A blocked reader wakes within about one buffer
   period plus its scheduling delay and gets either one last full buffer
   or `ESP_ERR_INVALID_STATE`.
2. The reading task sees `ESP_ERR_INVALID_STATE`, leaves its loop and
   signals that it has exited (task notification, event group, ...).
3. Wait for that signal, then call `i2s_mic_deinit()`. `deinit()` returns
   `ESP_ERR_INVALID_STATE` while a read is still in progress.

If the reading task is also the task that calls `stop()` (as in the
loopback example), steps 2 and 3 are trivial.

### How much delay is tolerated

The driver holds up to `dma_buffer_count - 1` finished buffers for you.
With `dma_buffer_count = 6` and 30 ms buffers, your reader can fall about
150 ms behind, counted from the last time it caught up, before the driver
starts dropping the oldest buffer. Raise `dma_buffer_count` for more
headroom; each extra buffer costs `dma_buffer_size` bytes of DMA-capable
RAM.

### Data coherence

The I2S driver has no lock between the DMA engine and the CPU copy. A
reader that is already a full ring behind, and is then preempted for
about one buffer period in the middle of its copy, can receive a buffer
the DMA has started rewriting. No error is returned in that case, but the
overflow count is already rising, which is why a rising count means
"audio unreliable". i2s_mic does not promise untorn buffers.

---

## Examples

| Example | Transport | Board type | Notes |
|---|---|---|---|
| `examples/uart-jtag` | Native USB-Serial-JTAG | ESP32-C3/S3/C6 boards with no separate UART bridge chip | Single cable, no extra hardware |
| `examples/uart` | UART0 (shared with console) | Classic ESP32 boards with a CP2102/CH340-style bridge chip | Single cable, no extra hardware |
| `examples/loopback_example` | none (on-device) | ESP32, ESP32-C3 | Records with `i2s_mic`, plays back with `i2s_spk` |

The streaming examples capture from an I2S mic and stream 16-bit PCM to a
PC script that plays it back live or records it to a WAV file.

---

## Testing

| Folder | What it is | How to run |
|---|---|---|
| `test/` | Unity test component: 16 `TEST_CASE`s for the read contract, loss accounting and stop/start behaviour | Built by `test_app/`, or add it to any project's `EXTRA_COMPONENT_DIRS` |
| `test_app/` | ESP-IDF project that runs every test on a board (no microphone needed) | `idf.py set-target esp32c3 build flash monitor` |
| `test_host/` | PC simulation of the ESP-IDF v6.0 I2S driver, for the wrapper's race conditions | `make` |

See `test_app/README.md` for wiring and how to read the results.

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
whatever the driver delivers; it has no way to know the underlying slot
content doesn't match what `channel_count = 1` implies.

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
   accordingly (`dma_buffer_size` must be an exact multiple of
   `channel_count * (bits_per_sample / 8)`, and must still fit in one DMA
   buffer: at most 4092 bytes, which is 511 stereo 32-bit frames).
2. After each `i2s_mic_read()`, keep only every other 32-bit sample
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


**DMA buffer size.** ESP-IDF caps one DMA buffer at 4092 bytes on most
targets and silently shrinks a larger request. `i2s_mic_init()` checks
the size the driver actually allocated and fails with
`ESP_ERR_INVALID_SIZE` (logging the largest size that fits) instead of
letting reads and your buffer sizes disagree.

**How the read stays safe during `stop()`.** `i2s_mic_read()` never passes
your timeout straight to the driver. It waits in slices of about two
buffer periods and re-checks whether capture is still running between
slices, and it checks every driver result for a short read. That is what
lets `stop()` end a read that passed `portMAX_DELAY`, and why a read
interrupted by `stop()` never reports success with partial data.

---

## Known Limitations

- Singleton only; no handle-based multi-instance API.
- MSB-justified/PCM/TDM slot formats, MCLK, and signal inversion are not
  supported.
- 24-bit slot width is not supported (`bits_per_sample` must be 16 or 32).
- One DMA buffer per `i2s_mic_read()` call.
- No coordination with a companion speaker component (e.g. automatic
  port-conflict detection on single-I2S-peripheral chips) — the
  application is responsible for correct port assignment.

---

## Requirements

- ESP-IDF v6.0.x. The driver behaviour this component relies on
  (`i2s_channel_read()` return values, `i2s_channel_disable()` waiting for
  an in-progress read, `i2s_channel_enable()` emptying the RX queue, the
  `on_recv_q_ovf` callback) is documented at the top of `i2s_mic.c`;
  re-check it before moving to a later ESP-IDF minor version.
- A target with `SOC_I2S_NUM >= 1` (see Chip Support above).

---

## License

MIT License — see LICENSE file.
