"""
Library for serial communication with the miniBot
"""

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

        self.expecting_start = True
        self.expecting_cmd = False
        self.expecting_data = False
        self.expecting_end = False

        self.start_4byte = 0xF000FF
        self.data = []
        self.end_4byte = 0xFFF0FF

        self.data_count = 0
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

    def send_cmd(self, cmd, data = []):
        _delay = 0.1
        while (self.waiting_response):
            pass
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
        while (self.waiting_response):
            if (self.ser.inWaiting() > 0):
                data = self.ser.read(self.ser.inWaiting())
                value = bytes_to_int32(data)
                if (value == self.start_4byte):
                    self.expecting_cmd = True
                    self.expecting_start = False
                elif (self.expecting_cmd == True):
                    self.expecting_cmd = False
                    self.expecting_data = True
                    self.data.append(value)
                elif (self.expecting_data == True):
                    if (self.data_count < cmd_len):
                        self.data_count += 1
                        self.data.append(value)
                    else:
                        self.data_count = 0
                        self.expecting_data = False
                        self.expecting_end = True
                elif (self.expecting_end == True):
                    self.waiting_response = False
        return self.data

def main():
    mini_bot_serial = MiniBotSerial()
    if (mini_bot_serial.ser.is_open):
        try:
            while True:
                cmd = 1
                data = [1, 2, 3, 4, 5, 6]
                mini_bot_serial.send_cmd(0x201)
                response = mini_bot_serial.read_response(6)
                print(response)
                time.sleep(1)
        except KeyboardInterrupt:
            print("Keyboard interrupt")
            print("Closing serial port...")
        finally:
            mini_bot_serial.close_port()

if __name__ == "__main__":
    main()


# import serial
# import time
# class MiniBotSerial():
#     def __init__(self, port = "/dev/serial_sdk", baudrate = "921600", retries = 5):
#         self.serial_status =  False
#         self.open_port(port, baudrate, retries)

#         self.expecting_start = True
#         self.expecting_cmd = False
#         self.expecting_data = False
#         self.expecting_end = False

#         self.read_success = False
#         self.waiting_response = False

#         self.start_4byte = 0xF0FF# 0xF0000FFF
#         self.data = []
#         self.end_4byte = 0xFF0F #0xFFFF0FFF
    
#     def open_port(self, port, baudrate, retries):
#         for i in range(retries):
#             try:
#                 self.ser = serial.Serial(port, baudrate)
#             except:
#                 pass

#                 if (self.ser.is_open) :
#                     print("Serial port opened successfully!")
#                     break
#                 if (i<retries-1):
#                     print("Failed to open serial port. Retrying... (%d/%d)"%(i+1, retries))
#                 else:
#                     print("Failed to open serial port.")
#                 time.sleep(1)

#     def close_port(self):
#         self.ser.close()
#         print("Serial port closed.")
    
#     def read_packet(self, cmd, len):
#         data_count = 0
#         while (self.waiting_response):
#             if self.ser.inWaiting() > 0:
#                 data = self.ser.read(self.ser.inWaiting())
#                 if (len(data) == 4):
#                     value = bytes_to_int32(data)
#                     # start 4-byte
#                     if (self.expecting_start == True):
#                         self.expecting_start = False
#                         if (value == self.start_4byte):
#                             self.expecting_cmd = True
#                             self.data.clear()
#                         else:
#                             self.read_failed()
#                             break
#                     # cmd 4-byte
#                     elif (self.expecting_cmd == True):
#                         if (value == cmd):
#                             self.expecting_data = True
#                             self.expecting_cmd = False
#                         else:
#                             self.read_failed()
#                             break
#                     # data
#                     elif (self.expecting_data == True):
#                         if (data_count < len):
#                             self.buffer.append(value)
#                             count += 1
#                         else:
#                             self.expecting_end = True
#                             self.expecting_data = False
#                     # end 4-byte
#                     elif (self.expecting_end == True):
#                         self.expecting_end = False
#                         if (value == self.end_4byte):
#                             self.read_success = True
#                             self.waiting_response = False
#                             break
#                         else:
#                             self.read_failed()
#                             break
#                 else:
#                     #len(data) != 4
#                     self.read_failed()
#                     break

#     def read_failed(self):
#         self.expecting_start = True
#         self.expecting_cmd = False
#         self.expecting_data = False
#         self.expecting_end = False
#         self.read_success = False
#         self.waiting_response = False
    
#     def write_packet(self, cmd, data):
#         self.ser.write(self.start_4byte)
#         time.sleep(0.1)
#         self.ser.write(cmd)
#         time.sleep(0.1)
#         for i in range(len(data)):
#             self.ser.write(data[i])
#             time.sleep(0.1)
#         self.ser.write(self.end_4byte)
#         time.sleep(0.1)
#         self.waiting_response = True
#         return 0

# def main():
#     mini_bot_serial = MiniBotSerial()
#     if (mini_bot_serial.ser.is_open):
#         try:
#             while True:
#                 cmd = 1
#                 data = [1, 2, 3, 4, 5, 6]
#                 # mini_bot_serial.write_packet(cmd, data)
#                 mini_bot_serial.ser.write(400)
#                 #mini_bot_serial.read_packet(cmd, len(data))
#                 if (mini_bot_serial.read_success):
#                     print(mini_bot_serial.data)
#                 time.sleep(0.1)
#         except KeyboardInterrupt:
#             print("Keyboard interrupt")
#             print("Closing serial port...")
#         finally:
#             mini_bot_serial.close_port()