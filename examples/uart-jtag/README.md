# i2s_mic streaming example (INMP441, ESP32-C3 / USB-Serial-JTAG)

Captures audio from an INMP441 MEMS microphone using the `i2s_mic`
component and streams it to a PC, which either plays it back in real time
or records it to a WAV file.

**This variant targets boards with no separate USB-to-UART bridge chip —
most ESP32-C3 devkits, where a single USB cable exposes the native
USB-Serial-JTAG peripheral and there's no physical UART0 pins connected to
anything.** It streams over that USB-Serial-JTAG endpoint directly instead
of a physical UART. If your board *does* have a separate CP2102/CH340-style
bridge (a second COM port appears when you plug in, wired to real UART0
pins), you can adapt `main/app_main.c` back to `driver/uart.h` — the
handshake shape is the same, just add back the baud-rate switch to
921600 for the fast streaming phase.

**Everything about the transport (handshake, sync bytes, header layout,
16-bit downconversion) lives in this example, not in `i2s_mic` itself.**
The component only knows about I2S and filled buffers.

## Wiring

| INMP441 pin | ESP32-C3 pin (default in app_main.c) |
|---|---|
| SCK (BCLK) | GPIO 7 |
| WS (LRCLK) | GPIO 8 |
| SD (DOUT)  | GPIO 9 |
| L/R        | GND |
| VDD        | 3.3V |
| GND        | GND |

Change `GPIO_BCK` / `GPIO_WS` / `GPIO_DATA` in `main/app_main.c` if your
board uses different pins.

**About L/R:** ESP-IDF's mono Philips-slot preset selects one fixed slot by
default. Which physical channel that maps to has varied across ESP-IDF
versions. If you get silence with L/R tied to GND, try VDD instead before
suspecting the wiring or code.

## Build and flash

This example vendors a local copy of `i2s_mic` under `components/i2s_mic/`
so it builds standalone without needing the component published to the
ESP Component Registry first. Requires ESP-IDF v6.0.x.

```bash
idf.py set-target esp32c3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

(On Windows this'll show up as a `COMx` port; on Linux, native USB-CDC
devices typically enumerate as `/dev/ttyACM*` rather than `/dev/ttyUSB*`.)

Once you see `=== READY, waiting for trigger ===` in the monitor, quit the
monitor (Ctrl+], or Ctrl+T then Ctrl+X) so the PC script can open the port
itself — the ESP32 and the monitor can't both hold it at once.

## Run the PC-side script

```bash
pip install pyserial
pip install sounddevice   # only needed for --mode play

# Record to a WAV file (Ctrl+C to stop and finalize the file):
python pc/mic_audio_rcv.py --port COM5 --mode record --out capture.wav

# Or listen live:
python pc/mic_audio_rcv.py --port COM5 --mode play
```

(`COM5` on Windows; `/dev/ttyACM0` or similar on Linux/macOS.)

## What's actually happening

1. `main/app_main.c` configures `i2s_mic` for 16 kHz, mono, **32-bit**
   samples — INMP441 outputs 24-bit audio MSB-justified in a 32-bit slot,
   and `i2s_mic` only supports 16- or 32-bit slots (24-bit is rejected at
   `init()`), so 32-bit is the correct choice here.
2. Filled buffers are handed from the ISR callback to a FreeRTOS queue; a
   worker task (`audio_sender_task`) downconverts each 32-bit sample to
   16-bit (`sample >> 16`) before writing it out over USB-Serial-JTAG. This
   halves the bandwidth and produces directly WAV-writable 16-bit PCM, at
   the cost of discarding INMP441's lowest-order bits. If your recordings
   are too quiet or too loud, adjust the shift amount or add explicit gain
   there — it's a one-line change and entirely independent of `i2s_mic`.
3. The ISR **re-posts a different buffer from the pool**, not the one it
   just delivered — that buffer is still awaiting conversion/transmission
   by `audio_sender_task`. If the pool is briefly empty (consumer running
   behind), the ISR simply skips re-posting for that cycle; `i2s_mic`
   surfaces the resulting drop through `overflow_cb`, logged periodically by
   `overflow_report_task` whenever you have logging enabled (logs are
   deliberately silenced once the binary PCM stream starts, to avoid
   corrupting it).
4. Bandwidth check: 16 kHz × 16-bit mono = 32,000 bytes/s of payload —
   trivial for a USB CDC endpoint. `usb_serial_jtag_write_bytes()`'s own
   blocking-when-the-TX-ring-is-full behavior is what ultimately paces the
   whole pipeline if the host ever falls behind, exactly as discussed in
   the design document.
5. `host_comm_init()` calls `usb_serial_jtag_driver_install()` to get
   interrupt-driven, buffered reads and writes on top of the same
   USB-Serial-JTAG cable that's already carrying the console's log output.
   These two coexist by design — ESP-IDF's console output uses a separate,
   lower-level path, which is why you already saw `=== READY ===` in the
   log before this driver was even installed. If you ever see console
   output and the data driver interfering with each other on your specific
   IDF version, the fallback is to disable
   `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG` in menuconfig — you'll
   lose console logging, but the example already disables logging before
   streaming starts anyway, so this only affects visibility of
   `=== READY ===` and the drop-count warnings.

## Known limitations of this example (not of `i2s_mic`)

- Streams indefinitely; there's no clean remote "stop" — just stop the PC
  script (Ctrl+C, which finalizes the WAV correctly) or reset the board.
- No compression, no VAD, no stereo — deliberately minimal, to keep the
  focus on how `i2s_mic` itself is used.
- Sharing the console channel for both ESP-IDF logging and the audio
  stream means you lose console logging for the duration of a capture,
  same tradeoff the JPEG example makes with UART0.
