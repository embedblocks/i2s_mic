# i2s_mic streaming example — UART-bridge variant

**For classic ESP32 (and other boards) with a separate USB-to-UART bridge
chip** — a second, distinct COM port appears when you plug in, wired to
the chip's real UART0 pins (true of most classic ESP32 devkits, using an
onboard CP2102, CH340, FTDI, etc.). This example reuses that same UART0 —
the one your programming/console cable already talks to — for the audio
stream too. No extra hardware needed, same single cable.

If your board instead has no separate bridge chip (the cable you flash
through is the chip's native USB-Serial-JTAG peripheral — common on
ESP32-C3/S3/C6 devkits), use the sibling example
**`i2s_mic_usb_jtag_example`** instead. The two are not interchangeable:
this one needs a real UART with a negotiable baud rate, which a native
USB-CDC endpoint is not.

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
| SCK (BCLK) | GPIO 16 |
| WS (LRCLK) | GPIO 17 |
| SD (DOUT)  | GPIO 18 |
| L/R        | GND |
| VDD        | 3.3V |
| GND        | GND |

Change `GPIO_BCK` / `GPIO_WS` / `GPIO_DATA` in `main/app_main.c` if your
board uses different pins. **Make sure L/R is actually tied to GND or
VDD, never left floating** — a floating L/R pin is a common source of
hum/noise on its own.

These GPIO defaults are classic-ESP32 numbers, chosen to avoid UART0's own
pins (GPIO1 TX / GPIO3 RX) and the usual SPI-flash pin range.

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

**On stereo capture:** this example requests STEREO and discards one slot
(`KEEP_SLOT`), the same defensive pattern used in `i2s_mic_usb_jtag_example`
— even though the specific mono-RX interleaving bug that motivated it was
diagnosed on ESP32-C3, not classic ESP32 (ESP-IDF's own documentation
suggests plain mono capture may work correctly on classic ESP32/S2). It's
kept here for consistency and because it costs almost nothing; if you've
verified plain mono works cleanly on your exact board, feel free to
simplify to `channel_count = 1` / `I2S_SLOT_MODE_MONO`.

## Build and flash

This example vendors a local copy of `i2s_mic` under `components/i2s_mic/`
so it builds standalone without needing the component published to the
ESP Component Registry first. Requires ESP-IDF v6.0.x.

```bash
idf.py set-target esp32       # or whichever target has your bridge chip
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
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

(`COM5` on Windows; `/dev/ttyUSB0` or similar on Linux/macOS.)

## What's actually happening

1. `main/app_main.c` configures `i2s_mic` for 16 kHz, **stereo**, **32-bit**
   samples — INMP441 outputs 24-bit audio MSB-justified in a 32-bit slot,
   and `i2s_mic` only supports 16- or 32-bit slots (24-bit is rejected at
   `init()`), so 32-bit is the correct choice there.
2. Filled buffers are handed from the ISR callback to a FreeRTOS queue; a
   worker task (`audio_sender_task`) keeps only `KEEP_SLOT` of each
   interleaved stereo pair and downconverts that 32-bit sample to 16-bit
   (`sample >> 16`) before writing it out over UART0. If your recordings
   are too quiet or too loud, adjust the shift amount or add explicit gain
   there — it's a one-line change and entirely independent of `i2s_mic`.
3. The ISR **re-posts a different buffer from the pool**, not the one it
   just delivered — that buffer is still awaiting conversion/transmission
   by `audio_sender_task`. If the pool is briefly empty (consumer running
   behind), the ISR simply skips re-posting for that cycle; `i2s_mic`
   surfaces the resulting drop through `overflow_cb`, logged periodically
   by `overflow_report_task` whenever logging is enabled (it's silenced
   once the binary PCM stream starts, to avoid corrupting it).
4. `host_comm_init()` reuses **UART0** — the same peripheral the console
   and programming connection already use — via `uart_vfs_dev_use_driver()`
   to take over its full interrupt-driven driver. The handshake connects at
   a safe 115200 baud, waits for a validated trigger byte, then switches to
   460800 for the actual streaming, matching the classic
   "slow-handshake-then-fast-bulk-transfer" pattern.
5. The trigger read loops until it sees the specific `TRIGGER_BYTE` value
   (`0xA5`) rather than accepting the first byte it gets. This matters
   because the header is only ever sent once — if a stray byte were
   accepted as the trigger prematurely, the PC script would be left
   waiting forever for a header that already went out before it connected.
6. Bandwidth check: 16 kHz × 16-bit mono = 32,000 bytes/s of payload,
   comfortably under UART @ 460800 baud's ~46,000 bytes/s raw capacity —
   `uart_write_bytes()`'s own blocking-when-the-TX-ring-is-full behavior is
   what ultimately paces the pipeline if the host ever falls behind.
   460800 was chosen over the more common 921600 because several cheap
   USB-serial bridge chips (CH340/CP2102 clones especially) don't reliably
   honor 921600 on the PC/driver side — the ESP can transmit at that rate
   just fine, but the PC's OS/driver may silently fail to switch to match
   it, causing the two ends to talk past each other with zero visible
   error. 460800 is much more consistently supported while still leaving
   healthy headroom over the required 32,000 bytes/s.

## Known limitations of this example (not of `i2s_mic`)

- Streams indefinitely; there's no clean remote "stop" — just stop the PC
  script (Ctrl+C, which finalizes the WAV correctly) or reset the board.
- No compression, no VAD — deliberately minimal, to keep the focus on how
  `i2s_mic` itself is used.
- Sharing UART0 for both the ESP-IDF console and the audio stream means
  you lose console logging for the duration of a capture.
