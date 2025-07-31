"""
UART communication on Raspberry Pi using Python
Writing to /dev/ttyAMA10 at 115200 baud
"""
import serial
from time import sleep

# Configure serial port
SERIAL_PORT = "/dev/serial0"
BAUD_RATE = 115200

try:
    # Initialize serial connection
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )
    
    print(f"Serial port {SERIAL_PORT} opened at {BAUD_RATE} baud")
    print("Type messages to send. Press Ctrl+C to exit")
    
    while True:
        # Get user input
        message = input("Send: ")
        
        # Send message over serial
        ser.write((message + '\n').encode())
        print(f"Sent: {message}")
        
        # Short delay to allow device response
        sleep(0.01)
        
        # Check for incoming data
        if ser.in_waiting > 0:
            received = ser.read(ser.in_waiting).decode().strip()
            print(f"Received: {received}")

except KeyboardInterrupt:
    print("\nProgram terminated by user")
except Exception as e:
    print(f"Error: {str(e)}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")