# i2s_mic UART streaming example (INMP441)

Captures audio from an INMP441 MEMS microphone using the `i2s_mic`
component and streams it over UART to a PC, which either plays it back in
real time or records it to a WAV file. Structurally mirrors the
JPEG-over-UART example: slow handshake baud → trigger byte → fast
streaming baud → binary header → continuous binary payload.

**Everything about the transport (handshake, sync bytes, header layout,
16-bit downconversion) lives in this example, not in `i2s_mic` itself.**
The component only knows about I2S and filled buffers.

## Wiring

| INMP441 pin | ESP32 pin (default in app_main.c) |
|---|---|
| SCK (BCLK) | GPIO 14 |
| WS (LRCLK) | GPIO 15 |
| SD (DOUT)  | GPIO 32 |
| L/R        | GND |
| VDD        | 3.3V |
| GND        | GND |

Change `GPIO_BCK` / `GPIO_WS` / `GPIO_DATA` in `main/app_main.c` if your
board uses different pins.

**About L/R:** ESP-IDF's mono Philips-slot preset selects one fixed slot by
default, and which physical channel that maps to has varied across ESP-IDF
versions. If you get silence with L/R tied to GND, try VDD instead before
suspecting the wiring or code.

## Build and flash

This example vendors a local copy of `i2s_mic` under `components/i2s_mic/`
so it builds standalone without needing the component published to the
ESP Component Registry first. Requires ESP-IDF v6.0.x.

```bash
idf.py set-target esp32          # or esp32s3 / esp32c3 / esp32c6
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

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

(`COM5` on Windows; something like `/dev/ttyUSB0` or `/dev/cu.usbserial-*`
on Linux/macOS.)

## What's actually happening

1. `main/app_main.c` configures `i2s_mic` for 16 kHz, mono, **32-bit**
   samples — INMP441 outputs 24-bit audio MSB-justified in a 32-bit slot,
   and `i2s_mic` only supports 16- or 32-bit slots (24-bit is rejected at
   `init()`), so 32-bit is the correct choice here.
2. Filled buffers are handed from the ISR callback to a FreeRTOS queue; a
   worker task (`audio_sender_task`) downconverts each 32-bit sample to
   16-bit (`sample >> 16`) before writing it to UART. This halves the
   bandwidth and produces directly WAV-writable 16-bit PCM, at the cost of
   discarding INMP441's lowest-order bits. If your recordings are too quiet
   or too loud, adjust the shift amount or add explicit gain there — it's a
   one-line change and entirely independent of `i2s_mic`.
3. The ISR **re-posts a different buffer from the pool**, not the one it
   just delivered — that buffer is still awaiting conversion/transmission
   by `audio_sender_task`. If the pool is briefly empty (consumer running
   behind), the ISR simply skips re-posting for that cycle; `i2s_mic`
   surfaces the resulting drop through `overflow_cb`, logged periodically by
   `overflow_report_task` whenever you have logging enabled (logs are
   deliberately silenced once the binary PCM stream starts, to avoid
   corrupting it).
4. Bandwidth check: 16 kHz × 16-bit mono = 32,000 bytes/s of payload,
   comfortably under UART @ 921600 baud's ~92,000 bytes/s raw capacity —
   the `uart_write_bytes()` call's own blocking-when-the-TX-ring-is-full
   behavior is what ultimately paces the whole pipeline, exactly as
   discussed in the design document.

## Known limitations of this example (not of `i2s_mic`)

- Streams indefinitely; there's no clean remote "stop" — just stop the PC
  script (Ctrl+C, which finalizes the WAV correctly) or reset the board.
- No compression, no VAD, no stereo — deliberately minimal, to keep the
  focus on how `i2s_mic` itself is used.
- Sharing UART0 for both the ESP-IDF console and the audio stream means you
  lose console logging for the duration of a capture, same tradeoff the
  JPEG example makes.
