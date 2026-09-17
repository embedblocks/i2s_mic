#!/usr/bin/env python3
"""
mic_audio_rcv.py — companion PC-side script for the i2s_mic streaming
example (USB-Serial-JTAG variant, for boards with no separate UART bridge
chip — e.g. most ESP32-C3 devkits with a single USB port).

The wire format (handshake sequence, sync bytes, header layout, 16-bit PCM
payload) is defined jointly by this script and the ESP32 example's
app_main.c. The i2s_mic *component* has no involvement in or knowledge of
any of this — it only produces filled buffers inside the firmware.

There's no baud rate to negotiate here: the ESP's USB-Serial-JTAG endpoint
is a real USB CDC device, not a UART, so pyserial's `baudrate` is accepted
but ignored by the firmware. If your board instead has a separate
CP2102/CH340-style UART bridge (a second, distinct COM port appears when
you plug in), use the true-UART version of this script/firmware instead,
which does negotiate a faster baud rate for streaming.

Usage:
    python mic_audio_rcv.py --port COM5 --mode record --out capture.wav
    python mic_audio_rcv.py --port COM5 --mode play

Dependencies:
    pip install pyserial
    pip install sounddevice   # only required for --mode play
"""

import argparse
import struct
import sys
import time
import wave

import serial

SYNC_BYTE = 0xAA
MAGIC = 0xC0FFEE01
SERIAL_BAUD = 115200  # ignored by native USB-CDC; pyserial's API requires a value anyway
CHUNK_BYTES = 4096  # serial read granularity; independent of the ESP's own DMA buffer size

# audio_header_t on the wire: sync[3] + magic(u32) + sample_rate(u32)
#                              + bits_per_sample(u16) + channel_count(u8)
HEADER_TAIL_FMT = "<IHB"          # everything after the 3-byte sync + 4-byte magic
HEADER_TAIL_SIZE = struct.calcsize(HEADER_TAIL_FMT)


def wait_for_ready(ser):
    print("[1] Waiting for READY...")
    while True:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(f"  ESP: {line}")
        if "READY" in line:
            return


def send_trigger(ser):
    print("[2] Sending trigger...")
    ser.write(b"\x01")
    ser.flush()
    ser.timeout = 5
    for _ in range(10):
        line = ser.readline().decode(errors="replace").strip()
        if line:
            print(f"  ESP: {line}")
        if "Trigger" in line:
            print("  Trigger acknowledged")
            return
    print("  Warning: no trigger acknowledgement seen — continuing anyway")


def read_exact(ser, size):
    data = bytearray()
    start = time.time()
    while len(data) < size:
        chunk = ser.read(size - len(data))
        if not chunk:
            if time.time() - start > 10:
                raise TimeoutError(f"Timeout waiting for header bytes: {len(data)}/{size}")
            continue
        data.extend(chunk)
    return bytes(data)


def find_sync_and_magic(ser):
    """Scan byte-by-byte for 0xAA 0xAA 0xAA followed by the 4-byte magic."""
    search_buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            raise TimeoutError("No sync pattern received")
        search_buf.append(b[0])
        if len(search_buf) >= 7:
            tail = search_buf[-7:]
            if (
                tail[0] == SYNC_BYTE
                and tail[1] == SYNC_BYTE
                and tail[2] == SYNC_BYTE
                and struct.unpack("<I", tail[3:7])[0] == MAGIC
            ):
                return


def read_header(ser):
    find_sync_and_magic(ser)
    rest = read_exact(ser, HEADER_TAIL_SIZE)
    sample_rate, bits_per_sample, channel_count = struct.unpack(HEADER_TAIL_FMT, rest)
    return sample_rate, bits_per_sample, channel_count


def run_record(ser, sample_rate, channel_count, out_path):
    wf = wave.open(out_path, "wb")
    wf.setnchannels(channel_count)
    wf.setsampwidth(2)  # 16-bit
    wf.setframerate(sample_rate)

    print(f"[+] Recording to {out_path} — Ctrl+C to stop")
    bytes_per_frame = 2 * channel_count
    pending = bytearray()
    total = 0
    try:
        while True:
            chunk = ser.read(CHUNK_BYTES)
            if chunk:
                pending += chunk
                usable_len = len(pending) - (len(pending) % bytes_per_frame)
                if usable_len:
                    wf.writeframes(bytes(pending[:usable_len]))
                    total += usable_len
                    del pending[:usable_len]
                    print(f"\r  {total / 1024:.1f} KiB captured", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        wf.close()
        print(f"\n[+] Saved {out_path} ({total} bytes of PCM, "
              f"{total / (sample_rate * channel_count * 2):.1f}s)")


def run_play(ser, sample_rate, channel_count):
    try:
        import sounddevice as sd
    except ImportError:
        print("Realtime playback needs the 'sounddevice' package: pip install sounddevice")
        sys.exit(1)

    stream = sd.RawOutputStream(samplerate=sample_rate, channels=channel_count, dtype="int16")
    stream.start()
    print("[+] Playing back live — Ctrl+C to stop")

    # ser.read() can return a short, non-frame-aligned chunk (e.g. an odd
    # number of bytes) once the read timeout elapses mid-sample. Writing a
    # misaligned buffer to a RawOutputStream raises an exception that would
    # otherwise silently kill this loop after the first short read — so we
    # hold any leftover partial-frame byte over to be prepended to the next
    # chunk instead of writing it immediately.
    bytes_per_frame = 2 * channel_count  # 16-bit samples
    pending = bytearray()
    try:
        while True:
            chunk = ser.read(CHUNK_BYTES)
            if not chunk:
                continue
            pending += chunk
            usable_len = len(pending) - (len(pending) % bytes_per_frame)
            if usable_len:
                stream.write(bytes(pending[:usable_len]))
                del pending[:usable_len]
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop()
        stream.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--mode", choices=["play", "record"], default="record")
    ap.add_argument("--out", default="capture.wav", help="Output WAV path for --mode record")
    args = ap.parse_args()

    ser = serial.Serial(args.port, SERIAL_BAUD, timeout=30)
    ser.dtr = False
    ser.rts = False

    wait_for_ready(ser)
    ser.reset_input_buffer()
    send_trigger(ser)

    print("[3] Waiting for header...")
    sample_rate, bits_per_sample, channel_count = read_header(ser)
    print(f"[4] Header: {sample_rate} Hz, {bits_per_sample}-bit, {channel_count} ch")

    if bits_per_sample != 16:
        print(f"This script only decodes the 16-bit PCM wire format the example sends "
              f"(got {bits_per_sample}-bit in the header).")
        sys.exit(1)

    ser.timeout = 1  # short read timeout once streaming, so Ctrl+C is responsive

    if args.mode == "record":
        run_record(ser, sample_rate, channel_count, args.out)
    else:
        run_play(ser, sample_rate, channel_count)


if __name__ == "__main__":
    main()
