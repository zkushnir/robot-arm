"""Simple test script to check Arduino connection"""
import serial
import time

PORT = "COM7"
BAUD = 115200

print(f"Attempting to connect to {PORT} at {BAUD} baud...")
print("Make sure Arduino IDE is CLOSED (especially Serial Monitor)")
print()

try:
    # Try to open the serial port
    ser = serial.Serial(PORT, baudrate=BAUD, timeout=1)
    print(f"✓ Port {PORT} opened successfully!")

    # Wait for Arduino to reset (happens when serial connection is made)
    print("Waiting 2 seconds for Arduino to initialize...")
    time.sleep(2)

    # Clear any startup messages
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # Send GET command to check if Arduino responds
    print("\nSending 'GET' command...")
    ser.write(b"GET\n")
    time.sleep(0.1)

    # Read response
    print("Waiting for response...")
    response_count = 0
    for i in range(10):
        line = ser.readline()
        if line:
            response_count += 1
            decoded = line.decode('utf-8', errors='replace').strip()
            print(f"  Arduino: {decoded}")

    if response_count == 0:
        print("\n⚠ No response from Arduino!")
        print("This might mean:")
        print("  - Wrong firmware uploaded")
        print("  - Arduino is stuck/not running")
        print("  - Baud rate mismatch")
    else:
        print(f"\n✓ SUCCESS! Received {response_count} line(s) from Arduino")

    ser.close()
    print("\nPort closed.")

except serial.SerialException as e:
    print(f"✗ ERROR: Could not open {PORT}")
    print(f"  Error message: {e}")
    print("\nPossible causes:")
    print("  1. Arduino IDE Serial Monitor is open (CLOSE IT!)")
    print("  2. Another program is using the port")
    print("  3. Wrong COM port (check Device Manager)")
    print("  4. USB cable disconnected")
    print("\nTo find available ports:")
    print("  - Open Device Manager")
    print("  - Look under 'Ports (COM & LPT)'")
    print("  - Find 'Arduino' or 'USB Serial Device'")

except Exception as e:
    print(f"✗ Unexpected error: {e}")
    import traceback
    traceback.print_exc()
