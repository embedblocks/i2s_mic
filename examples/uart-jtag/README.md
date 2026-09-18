# i2s_mic streaming example — USB-Serial-JTAG variant

**For ESP32-C3, ESP32-S3, ESP32-C6, and other boards with a native
USB-Serial-JTAG peripheral and no separate USB-to-UART bridge chip** — the
single USB cable you already flash/monitor through is used for the audio
stream too. No extra hardware needed.

If your board instead has a separate CP2102/CH340-style bridge chip (a
second, distinct COM port appears when you plug in — true of most classic
ESP32 devkits), use the sibling example **`i2s_mic_uart_bridge_example`**
instead. The two are not interchangeable: this one only works on chips
with a native USB-Serial-JTAG peripheral, and reuses the exact same cable
your programming/console connection already uses.

Captures audio from an I2S digital MEMS microphone using the `i2s_mic`
component and streams it to a PC, which either plays it back in real time
or records it to a WAV file. Developed and tested with an **INMP441**, and
also confirmed working — unmodified — with an **MSM261S4030H0**. See
"Microphone compatibility" below.

**Everything about the transport (handshake, sync bytes, header layout,
16-bit downconversion) lives in this example, not in `i2s_mic` itself.**
The component only knows about I2S and filled buffers.

## Wiring

| Mic pin | Example default (adjust for your board) |
|---|---|
| SCK (BCLK) | GPIO 7 |
| WS (LRCLK) | GPIO 8 |
| SD (DOUT)  | GPIO 9 |
| L/R        | GND |
| VDD        | 3.3V |
| GND        | GND |

Change `GPIO_BCK` / `GPIO_WS` / `GPIO_DATA` in `main/app_main.c` if your
board uses different pins. **Make sure L/R is actually tied to GND or
VDD, never left floating** — a floating L/R pin is a common source of
hum/noise on its own, independent of anything else in this README.

## Microphone compatibility

`i2s_mic` has no INMP441-specific logic at all — it's a generic I2S
receiver that copies whatever the DMA hands it. Any I2S digital MEMS mic
that speaks the standard Philips format with 24-bit samples MSB-justified
in a 32-bit slot (the same layout the INMP441 uses) should work without
code changes. This has been directly confirmed with an **MSM261S4030H0**
in place of the INMP441, no changes needed. Other common parts using the
same format (e.g. ICS-43434, SPH0645) are likely compatible too, though
not directly tested here. INMP441 is used as the running example below
simply because it's the most widely documented part in this space.

**About mono capture — read this if your recording is distorted or
hum-only.** On at least one tested ESP32-C3 / ESP-IDF v6.0.x combination,
requesting `I2S_SLOT_MODE_MONO` for RX did **not** give a compact
single-slot stream — the DMA buffer still contained both I2S slots
interleaved, and only one of every two 32-bit words was real mic audio.
This example works around it by requesting **STEREO** explicitly and
discarding the unwanted slot in `audio_sender_task()`, controlled by the
`KEEP_SLOT` macro (0 = first/even word of each pair, 1 = second/odd word).
With L/R tied to GND, the real signal showed up in slot 0. If your
recording comes out silent or hum-only instead of clean, try `KEEP_SLOT 1`.

## Build and flash

This example vendors a local copy of `i2s_mic` under `components/i2s_mic/`
so it builds standalone without needing the component published to the
ESP Component Registry first. Requires ESP-IDF v6.0.x.

```bash
idf.py set-target esp32c3       # or esp32s3 / esp32c6
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Once you see `=== READY, waiting for trigger ===`, quit the monitor
(Ctrl+], or Ctrl+T then Ctrl+X) so the PC script can open the port itself —
the ESP32 and the monitor can't both hold it at once.

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

1. `main/app_main.c` configures `i2s_mic` for 16 kHz, **stereo**, **32-bit**
   samples — INMP441 outputs 24-bit audio MSB-justified in a 32-bit slot,
   and `i2s_mic` only supports 16- or 32-bit slots (24-bit is rejected at
   `init()`), so 32-bit is the correct choice there. Stereo (rather than
   mono) is requested for the reason explained above.
2. Filled buffers are handed from the ISR callback to a FreeRTOS queue; a
   worker task (`audio_sender_task`) keeps only `KEEP_SLOT` of each
   interleaved stereo pair and downconverts that 32-bit sample to 16-bit
   (`sample >> 16`) before writing it out over USB-Serial-JTAG. If your
   recordings are too quiet or too loud, adjust the shift amount or add
   explicit gain there — it's a one-line change and entirely independent
   of `i2s_mic`.
3. The ISR **re-posts a different buffer from the pool**, not the one it
   just delivered — that buffer is still awaiting conversion/transmission
   by `audio_sender_task`. If the pool is briefly empty (consumer running
   behind), the ISR simply skips re-posting for that cycle; `i2s_mic`
   surfaces the resulting drop through `overflow_cb`, logged periodically
   by `overflow_report_task` whenever logging is enabled (it's silenced
   once the binary PCM stream starts, to avoid corrupting it).
4. `host_comm_init()` calls `usb_serial_jtag_driver_install()` to get
   interrupt-driven, buffered reads and writes on top of the same
   USB-Serial-JTAG cable that already carries the console's log output.
   These two coexist by design — ESP-IDF's console output uses a separate,
   lower-level path, which is why you already saw `=== READY ===` in the
   log before this driver was even installed.
5. The trigger read loops until it sees the specific `TRIGGER_BYTE` value
   (`0xA5`) rather than accepting the first byte it gets. This matters
   because the header is only ever sent once — if a stray byte were
   accepted as the trigger prematurely, the PC script would be left
   waiting forever for a header that already went out before it connected.

## Known limitations of this example (not of `i2s_mic`)

- Streams indefinitely; there's no clean remote "stop" — just stop the PC
  script (Ctrl+C, which finalizes the WAV correctly) or reset the board.
- No compression, no VAD — deliberately minimal, to keep the focus on how
  `i2s_mic` itself is used.
- Sharing the console channel for both logging and the audio stream means
  you lose console log visibility for the duration of a capture.
