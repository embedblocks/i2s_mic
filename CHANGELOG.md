# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).


## [0.2.0] - 2026-10-01

### Breaking changes
- Replaced the ISR callback model with a blocking read API. Removed
  `i2s_mic_request_buffer()`, `mic_buffer_ready_cb_t`, `mic_overflow_cb_t`,
  and the `cb`, `overflow_cb`, `user_ctx` and `max_pending_buffers` fields
  of `i2s_mic_config_t`. Applications now call `i2s_mic_read()` from their
  own task.
- `i2s_mic_init()` now takes `const i2s_mic_config_t *`.
- `i2s_mic_init()` fails with `ESP_ERR_INVALID_SIZE` when
  `dma_buffer_size` exceeds what the driver can allocate as one DMA buffer
  (4092 bytes on most targets), instead of the driver silently shrinking
  it.

### Added
- `i2s_mic_read()`: reads exactly one DMA buffer per call, all or nothing.
  Waits in short slices so `i2s_mic_stop()` always ends a blocked read,
  and reports a read interrupted by `stop()` as `ESP_ERR_INVALID_STATE`.
- `i2s_mic_get_overflow_count()`: number of DMA buffers the driver
  discarded because the reader fell behind, since the last `start()`.
- `i2s_mic_get_buffer_size()`: the exact length `i2s_mic_read()` requires.
- `test/` (Unity test component) and `test_app/` (runner project): hardware
  tests for the read contract, loss accounting, stop latency, CPU load and
  start/stop cycles.
- `test_host/`: PC simulation of the ESP-IDF v6.0 I2S RX driver for
  testing the wrapper's lifecycle races.

### Fixed
- The overflow count no longer rises during normal operation. 0.1.x
  counted the driver's `on_recv_q_ovf` event, which fired on every DMA
  buffer because nothing drained the driver's queue; no audio was
  actually lost.
- Examples no longer lose posted buffers when their free pool is briefly
  empty, which could stall capture permanently.
- Examples request 480-frame (30 ms) DMA buffers, which fit in one DMA
  buffer. 0.1.x examples requested 512 frames (4096 bytes), which the
  driver silently shrank to 511.

### Changed
- `i2s_mic_deinit()` refuses to run while a task is inside
  `i2s_mic_read()`.
- `i2s_mic_start()` discards audio left in the driver from a previous run.
- No user code runs in ISR context; the only ISR code is a counter
  increment.

## [0.1.3] - 2026-09-18
### Refactored
- All readme updated to show support for both ini and msm series
- Readme reformat for esp registry

## [0.1.2] - 2026-09-18

### Added
- `examples/uart`: streams audio to a PC over UART0,
  for classic ESP32 (and similar) boards with a separate USB-to-UART
  bridge chip (CP2102/CH340/FTDI, etc.). Reuses the same UART0 the
  programming/console cable already talks to — no extra hardware needed.
  Includes a companion PC-side Python script (`pc/mic_audio_rcv.py`) for
  live playback or WAV recording.

## [0.1.1] - 2026-09-18

### Added
- `examples/uar-jtag`: streams audio to a PC over native
  USB-Serial-JTAG, for ESP32-C3/S3/C6 (and similar) boards with no
  separate USB-to-UART bridge chip. Uses the same single USB cable already
  used for flashing/monitoring — no extra hardware needed. Includes a
  companion PC-side Python script (`pc/mic_audio_rcv.py`) for live
  playback or WAV recording.

## [0.1.0] - 2026-09-18

### Added
- Initial release of `i2s_mic`: a task-free, ISR-driven I2S microphone
  component for ESP-IDF v6.0.x, built on the modern `i2s_std` driver.
- Application-owned buffer model — `i2s_mic_request_buffer()` posts
  caller-allocated buffers; filled buffers are returned via
  `mic_buffer_ready_cb_t`, invoked inline from the I2S driver's ISR
  dispatch, with no internal FreeRTOS task.
- Dual overflow tracking via `mic_overflow_cb_t`: ESP-IDF's own internal
  DMA queue overflow and "no application buffer posted in time" are
  reported as two independent counters.
- Dual-context-safe `i2s_mic_request_buffer()`, usable from an ordinary
  task or from inside the ISR callback itself.
- Support for 16-bit and 32-bit sample widths, Philips I2S standard slot
  format, `I2S_ROLE_MASTER`.
- Full lifecycle API: `i2s_mic_init()`, `i2s_mic_start()`,
  `i2s_mic_stop()`, `i2s_mic_deinit()`.
