import serial
import time

# Replace with your correct serial port
serial_port = '/dev/ttyTHS1'  # Example for UART1 on Jetson Orin Nano

# Initialize serial communication at 9600 baud rate
ser = serial.Serial(serial_port, 9600, timeout=1)

while True:
    # Send data to Teensy
    ser.write(b"Hello from Jetson!\n")

    # Check if data is available from Teensy
    if ser.in_waiting > 0:
        received_data = ser.readline().decode('utf-8').strip()
        print(f"Received from Teensy: {received_data}")

    time.sleep(1)  # Wait 1 second before sending the next message