"""
Record the AS5600 stream from the ESP32 into a CSV file.

Usage:
    python capture_swing.py COM11                 (Windows)
    python capture_swing.py /dev/ttyUSB0          (Linux)
    python capture_swing.py COM11 --seconds 30 --out run2.csv

Close the Arduino Serial Monitor first: only one program can use the port.
"""

import argparse
import csv
import time

import numpy as np
import serial

parser = argparse.ArgumentParser()
parser.add_argument("port", help="serial port, e.g. COM11 or /dev/ttyUSB0")
parser.add_argument("--seconds", type=float, default=30.0, help="recording length")
parser.add_argument("--out", default="real_swing.csv", help="output CSV file")
parser.add_argument("--baud", type=int, default=115200)
args = parser.parse_args()


# On ESP32 dev boards the DTR/RTS lines are wired to the EN (reset) and BOOT
# pins. On Windows, opening a port asserts them by default, which can hold the
# ESP32 in reset so it sends nothing. Set both low BEFORE opening the port.
ser = serial.Serial()
ser.port = args.port
ser.baudrate = args.baud
ser.timeout = 0.1  # short, so the script never blocks for long
ser.dtr = False
ser.rts = False
ser.open()
 
with ser:
    # Wait for the first data line, showing any startup messages
    print(f"Connecting to {args.port}...")
    got_data = False
    t_end = time.time() + 5.0
    while time.time() < t_end and not got_data:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if line.startswith("#"):
            print("  ESP32 says:", line[1:].strip())
        elif "," in line:
            got_data = True
    if not got_data:
        raise SystemExit(
            "No data from the ESP32 within 5 s. Check:\n"
            "  - the Arduino Serial Monitor is closed\n"
            "  - the port name is right and the sketch is uploaded\n"
            "  - try pressing the board's EN/RESET button while this waits")
    print("  Receiving data.")
    ser.reset_input_buffer()
 
    print("\nHold the pendulum at ~10 degrees.")
    for n in (3, 2, 1):
        print(f"  recording in {n}...")
        time.sleep(1.0)
    ser.reset_input_buffer()
    print(f"RECORDING for {args.seconds:.0f} s -- release the pendulum now.")
 
    rows, skipped = [], 0
    end = time.time() + args.seconds
    while time.time() < end:
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if not line or line.startswith("#"):
            if "error" in line:
                skipped += 1
            continue
        parts = line.split(",")
        try:
            rows.append((int(parts[0]), int(parts[1])))
        except (ValueError, IndexError):
            skipped += 1  # partial or garbled line
 
with open(args.out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["micros", "raw"])
    w.writerows(rows)
 
# ---- Quick quality check ----------------------------------------------------
if len(rows) < 2:
    raise SystemExit("Almost no data received. Check the port and that the sketch is running.")
 
us = np.array([r[0] for r in rows], dtype=np.int64)
dt = np.diff(us) % 2**32  # survives micros() rollover
print(f"\nSaved {len(rows)} samples to {args.out}")
print(f"Sample rate: {1e6 / np.median(dt):.0f} Hz, largest gap {dt.max() / 1000:.1f} ms")
if skipped:
    print(f"Skipped {skipped} bad lines")
if dt.max() > 3 * np.median(dt):
    print("WARNING: some samples were dropped. Fine for the period, "
          "but worth checking if it happens a lot.")