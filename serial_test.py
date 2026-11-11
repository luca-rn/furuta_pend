"""
Minimal serial test: is the ESP32 sending anything, and does Python receive it?
 
Usage:  python serial_test.py COM11
        python serial_test.py COM11 115200     (other baud rate)
"""
 
import sys
import serial
 
port = sys.argv[1]
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600
 
print(f"Opening {port} at {baud} baud...")
ser = serial.Serial(port, baud, timeout=0.5)
print("Opened. Reading 10 times, 0.5 s each:")
 
for i in range(10):
    data = ser.read(64)  # returns after 64 bytes or 0.5 s, whichever is first
    print(f"  read {i}: {len(data):2d} bytes  {data[:40]!r}")
 
ser.close()
print("Done.")