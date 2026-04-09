import serial

try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print("Port opened successfully")
    ser.close()
except Exception as e:
    print(f"Failed to open port: {e}")
