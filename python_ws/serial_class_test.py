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
        print(struct.unpack("<i", data))
        return struct.unpack("<i", data)[0]

def int32_to_bytes(int32, is_big_endian = 0):
    if (is_big_endian == 1):
        return struct.pack(">i", int32)
    else:
        # little-endian byte order
        return struct.pack("<i", int32)

import serial
import time

class MiniBotSerial():
    def __init__(self, port = "/dev/serial_sdk", baudrate = "921600", retries = 5):
        self.open_port(port, baudrate, retries)

        self.start_4byte = 0xF0FF #0xF0000FFF
        self.data = []
        self.end_4byte = 0xFF0F #0xFFFF0FFF

        self.waiting_response = False
    
    def open_port(self, port, baudrate, retries):
        for i in range(retries):
            try:
                self.ser = serial.Serial(port, baudrate)
            except:
                pass

            if (self.ser.is_open) :
                print("Serial port opened successfully!")
                break
            if (i<retries-1):
                print("Failed to open serial port. Retrying... (%d/%d)"%(i+1, retries))
            else:
                print("Failed to open serial port.")
            time.sleep(1)

    def send_cmd(self, cmd):
        while (self.waiting_response):
            pass
        self.ser.write(int32_to_bytes(self.start_4byte))
        time.sleep(0.1)
        self.ser.write(int32_to_bytes(cmd))
        time.sleep(0.1)
        self.ser.write(int32_to_bytes(self.end_4byte))
        time.sleep(0.1)
        self.waiting_response = True

    def read_response(self):
        while (self.waiting_response):
            if (self.ser.inWaiting() > 0):
                data = self.ser.read(self.ser.inWaiting())
                value = bytes_to_int32(data)
                self.waiting_response = False
        return value

def main():
    mini_bot_serial = MiniBotSerial()
    if (mini_bot_serial.ser.is_open):
        try:
            while True:
                cmd = 1
                data = [1, 2, 3, 4, 5, 6]
                mini_bot_serial.send_cmd(0x201)
                response = mini_bot_serial.read_response()
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Keyboard interrupt")
            print("Closing serial port...")
        finally:
            mini_bot_serial.close_port()

if __name__ == "__main__":
    main()