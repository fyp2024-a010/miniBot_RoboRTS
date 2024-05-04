"""
Library for serial communication with the miniBot
"""

import serial
import time
from number_lib import *

class MiniBotSerial():
    def __init__(self, port = "/dev/serial_sdk", baudrate = "921600", retries = 5):
        self.open_port(port, baudrate, retries)

        self.expecting_start = True
        self.expecting_cmd = False
        self.expecting_data = False
        self.expecting_end = False

        self.read_success = False
        self.waiting_response = False

        self.start_4byte = 0xF0000FFF
        self.data = []
        self.end_4byte = 0xFFFF0FFF
        self.get_port_status()
    
    def open_port(self, port, baudrate, retries):
        for i in range(retries):
            self.ser = serial.Serial(port, baudrate)
            if (self.ser.is_open) :
                print("Serial port opened successfully!")
                break
            elif (i<retries-1):
                print("Failed to open serial port. Retrying... (%d/%d)"%(i+1, retries))
            else:
                print("Failed to open serial port.")
            time.sleep(1)

    def close_port(self):
        self.ser.close()
        print("Serial port closed.")
    
    def read_packet(self, cmd, len):
        data_count = 0
        while (self.waiting_response):
            if self.ser.inWaiting() > 0:
                data = self.ser.read(self.ser.inWaiting())
                if (len(data) == 4):
                    value = bytes_to_int32(data)
                    # start 4-byte
                    if (self.expecting_start == True):
                        self.expecting_start = False
                        if (value == self.start_4byte):
                            self.expecting_cmd = True
                            self.data.clear()
                        else:
                            self.read_failed()
                            break
                    # cmd 4-byte
                    elif (self.expecting_cmd == True):
                        if (value == cmd):
                            self.expecting_data = True
                            self.expecting_cmd = False
                        else:
                            self.read_failed()
                            break
                    # data
                    elif (self.expecting_data == True):
                        if (data_count < len):
                            self.buffer.append(value)
                            count += 1
                        else:
                            self.expecting_end = True
                            self.expecting_data = False
                    # end 4-byte
                    elif (self.expecting_end == True):
                        self.expecting_end = False
                        if (value == self.end_4byte):
                            self.read_success = True
                            self.waiting_response = False
                            break
                        else:
                            self.read_failed()
                            break
                else:
                    #len(data) != 4
                    self.read_failed()
                    break

    def read_failed(self):
        self.expecting_start = True
        self.expecting_cmd = False
        self.expecting_data = False
        self.expecting_end = False
        self.read_success = False
        self.waiting_response = False
    
    def write_packet(self, cmd, data):
        self.ser.write(self.start_4byte)
        self.ser.write(cmd)
        for i in range(len(data)):
            self.ser.write(data[i])
        self.ser.write(self.end_4byte)
        self.waiting_response = True
        return 0

def main():
    mini_bot_serial = MiniBotSerial()
    if (mini_bot_serial.ser.is_open):
        try:
            while True:
                cmd = 1
                data = [1, 2, 3, 4, 5, 6]
                mini_bot_serial.write_packet(cmd, data)
                mini_bot_serial.read_packet(cmd, len(data))
                if (mini_bot_serial.read_success):
                    print(mini_bot_serial.buffer)
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("Keyboard interrupt")
            print("Closing serial port...")
        finally:
            mini_bot_serial.close_port()

if __name__ == "__main__":
    main()