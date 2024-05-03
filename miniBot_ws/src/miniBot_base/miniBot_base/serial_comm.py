"""
Library for serial communication with the miniBot
"""

import serial
from number_lib import *

class MiniBotSerial():
    def __init__(self, port = "/dev/serial_sdk", baudrate = "921600"):
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port=self.port, baudrate=self.baudrate)
        self.expecting_new_packet = True
        self.expecting_cmd = True
        self.start_byte = 14
        self.end_byte = 15
        self.read_success = False
        self.waiting_response = False
        self.buffer = []
        self.get_port_status()

    def get_port_status(self):
        if self.ser.is_open:
            print("Serial port opened successfully!")
            return True
        else:
            print("Failed to open serial port!")
            return False

    def close_port(self):
        self.ser.close()
        print("Serial port closed.")
    
    def read_packet(self, cmd):
        while (self.waiting_response):
            if self.ser.inWaiting() > 0:
                data = self.ser.read(self.ser.inWaiting())
                if (len(data) == 1):
                    # status (start and end byte)
                    if (self.expecting_new_packet):
                        if (data == self.start_byte):
                            self.read_success = False
                            self.expecting_new_packet = False
                            self.expecting_cmd = True
                        else:
                            self.read_success = False
                            self.read_reset()
                            break
                    else:
                        if (data == self.end_byte):
                            self.read_success = True
                            self.read_reset()
                            break
                        else:
                            self.read_success = False
                            self.read_reset()
                            break
                    return data
                elif (len(data) == 2):
                    # cmd
                    if (self.expecting_cmd):
                        value = bytes_to_uint16(data)
                        if (value == cmd):
                            self.buffer.append(value)
                        else:
                            self.read_success = False
                            self.read_reset()
                            break
                    else:
                        self.read_success = False
                        self.read_reset()
                        break
                elif (len(data) == 4):
                    # data
                    if (self.expecting_cmd == False and self.expecting_new_packet == False):
                        self.buffer.append(value)
                    else:
                        self.read_success = False
                        self.read_reset()
                        break
                else:
                    self.read_success = False
                    self.read_reset()
                    break

    def read_reset(self):
        self.buffer.clear()
        self.expecting_new_packet = True
        self.expecting_cmd = False
        self.waiting_response = False
    
    def write_packet(self, cmd, data):
        self.ser.write(self.start_byte)
        self.ser.write(cmd)
        for i in range(len(data)):
            self.ser.write(data[i])
        self.ser.write(self.start_byte)
        self.waiting_response = True
        return 0