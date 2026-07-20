import serial
import sys

PORT = "COM7"
BAUD = 9600

try:
    ser = serial.Serial(PORT, BAUD, timeout=2)
    print(f"Connected to {PORT}. Reading pin states (Ctrl+C to stop)...\n")
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line:
            print(line)
except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    try:
        ser.close()
    except:
        pass
