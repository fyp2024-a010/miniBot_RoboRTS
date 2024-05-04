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

import serial
import time

class MiniBotSerial():
    def __init__(self, port = "/dev/serial_sdk", baudrate = "921600", retries = 5):
        self.open_port(port, baudrate, retries)

        self.expecting_start = True
        self.expecting_cmd = False
        self.expecting_data = False
        self.expecting_end = False

        self.start_4byte = 0xF000FF
        self.buffer = []
        self.end_4byte = 0xFFF0FF

        self.response_count = 0
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
    
    def close_port(self):
        if (self.ser.is_open):
            self.ser.close()
            print("Serial port closed.")

    def send_cmd(self, cmd, data):
        _delay = 0.0001
        while (self.waiting_response):
            pass
        print("sending...")
        self.ser.write(int32_to_bytes(self.start_4byte))
        print(self.start_4byte)
        time.sleep(_delay)
        self.ser.write(int32_to_bytes(cmd))
        print(cmd)
        time.sleep(_delay)
        for i in range(len(data)):
            self.ser.write(int32_to_bytes(data[i]))
            print(data[i])
            time.sleep(_delay)
        self.ser.write(int32_to_bytes(self.end_4byte))
        print(self.end_4byte)
        time.sleep(_delay)
        self.waiting_response = True

    def read_response(self, cmd_len):
        print("receiving...")
        self.buffer = []
        while (self.waiting_response):
            if (self.ser.inWaiting() > 0):
                data = self.ser.read(self.ser.inWaiting())
                value = bytes_to_int32(data)
                print(value)
                if (self.expecting_start):
                    if (value == self.start_4byte):
                        self.expecting_start = False
                        self.expecting_cmd = True
                elif (self.expecting_cmd):
                    self.buffer.append(value)
                    self.expecting_cmd = False
                    self.expecting_data = True
                elif (self.expecting_data):
                    self.response_count += 1
                    self.buffer.append(value)
                    if (self.response_count == cmd_len):
                        self.response_count = 0
                        self.expecting_data = False
                        self.expecting_end = True
                elif (self.expecting_end):
                    if (value == self.end_4byte):
                        self.expecting_start = True
                        self.expecting_end = False
                        self.waiting_response = False
        return value

def main():
    mini_bot_serial = MiniBotSerial()
    if (mini_bot_serial.ser.is_open):
        try:
            while True:
                cmd = 0x201
                data = [1, 2, 3, 4, 5, 6]
                cmd_len = 6
                mini_bot_serial.send_cmd(cmd, data)
                response = mini_bot_serial.read_response(cmd_len)
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Keyboard interrupt")
            print("Closing serial port...")
        finally:
            mini_bot_serial.close_port()

if __name__ == "__main__":
    main()
