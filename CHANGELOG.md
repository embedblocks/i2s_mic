# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/).

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
