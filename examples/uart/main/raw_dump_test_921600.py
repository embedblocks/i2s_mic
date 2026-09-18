#!/usr/bin/env python3
"""
raw_dump_test_921600.py — matches minimal_trigger_test.c exactly (921600
baud, accept-any-byte trigger, no header/sync pattern to look for).

Usage: python raw_dump_test_921600.py --port COM5
"""
import argparse
import time
import serial

HANDSHAKE_BAUD = 115200
STREAM_BAUD = 921600  # must match minimal_trigger_test.c's hardcoded baud_rate
TRIGGER_BYTE = 0x01    # minimal_trigger_test.c accepts ANY byte; value doesn't matter

ap = argparse.ArgumentParser()
ap.add_argument("--port", required=True)
args = ap.parse_args()

ser = serial.Serial(args.port, HANDSHAKE_BAUD, timeout=1)
ser.dtr = False
ser.rts = False

print("[1] Sending trigger...")
ser.write(bytes([TRIGGER_BYTE]))
ser.flush()

time.sleep(0.7)
ser.reset_input_buffer()
ser.baudrate = STREAM_BAUD
time.sleep(0.1)
print(f"    (pyserial reports baudrate={ser.baudrate} after the switch)")

print(f"[2] Switched to {STREAM_BAUD} baud. Dumping raw bytes for 5 seconds...")
end = time.time() + 5
total = 0
first_chunk_shown = False
while time.time() < end:
    chunk = ser.read(64)
    if chunk:
        total += len(chunk)
        if not first_chunk_shown:
            print("First bytes received:", chunk.hex(" "))
            try:
                print("As text:", chunk.decode(errors="replace"))
            except Exception:
                pass
            first_chunk_shown = True

print(f"[3] Done. Total bytes received in 5s: {total}")
if total == 0:
    print("    -> Nothing arrived. Either the trigger never reached the firmware,")
    print("       or minimal_trigger_test.c wasn't actually the firmware running")
    print("       (double check you reflashed before this test).")
else:
    print("    -> Data arrived! Expected repeating pattern: 48 45 4c 4c 4f 21 21 ('HELLO!!')")
