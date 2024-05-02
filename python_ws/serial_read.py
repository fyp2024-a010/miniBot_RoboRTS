import serial
import time

# Replace with the actual port name of your device
port = '/dev/serial_sdk'  # Windows
# port = '/dev/ttyUSB0'  # Linux/Mac

# Set the baud rate according to your device
baudrate = 921600

# Configure the serial port
ser = serial.Serial(port=port, baudrate=baudrate)

import struct

def bytes_to_int32(data, is_big_endian = 0):
    if len(data) != 4:
        #raise ValueError("Input data must be exactly 4 bytes long. length = %d"%(len(data)))
        print("Input data must be exactly 4 bytes long. length = %d"%(len(data)))
        return None
    
    if (is_big_endian == 1):
        return struct.unpack(">i", data)[0]
    else:
        # little-endian byte order
        return struct.unpack("<i", data)[0]

def int32_to_bytes(int32, is_big_endian = 0):
    if (is_big_endian == 1):
        return struct.pack(">i", int32)
    else:
        # little-endian byte order
        return struct.pack("<i", int32)

# Check if the port is open
if ser.is_open:
    print("Serial port opened successfully!")
    last_time = time.time()

    try:
        while True:
            if (time.time() - last_time) > 1:
                data = int32_to_bytes(256)
                ser.write(data)
                last_time = time.time()
            # Check for incoming data (non-blocking)
            if ser.inWaiting() > 0:
                # Read data as bytes
                data = ser.read(ser.inWaiting())
                
                value = bytes_to_int32(data)
                # Decode data to a string (optional)
                # Assuming ASCII encoding
                # decoded_data = data.decode('ascii')
                
                # Print data (or perform further processing)
                print(value)
            else:
                # No data available, do something else...
                pass

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        # Close the serial port
        ser.close()
        print("Serial port closed.")

else:
    print("Failed to open serial port!")

