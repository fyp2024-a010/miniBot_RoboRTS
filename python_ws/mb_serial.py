import serial
import time
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

class MiniBotSerial():
    def __init__(self, port = "/dev/serial_sdk", baudrate = "921600", retries = 5, is_big_endian = 0):
        self.open_port(port, baudrate, retries)
        self.waiting_response = False
        self.is_big_endian = is_big_endian
    
    def open_port(self, port, baudrate, retries):
        for i in range(retries):
            try:
                self.ser = serial.Serial(port, baudrate)
            except:
                time.sleep(1)
                pass

            if (self.ser.is_open) :
                print("Serial port opened successfully!")
                break
            if (i<retries-1):
                print("Failed to open serial port. Retrying... (%d/%d)"%(i+1, retries))
            else:
                print("Failed to open serial port.")
    
    def close_port(self):
        if (self.ser.is_open):
            self.ser.close()
            print("Serial port closed.")
    
    def _send(self, value):
        while (self.waiting_response):
            pass
        self.ser.write(int32_to_bytes(value, self.is_big_endian))
        self.waiting_response = True
    
    def _read_response(self):
        while (self.waiting_response):
            if (self.ser.inWaiting() > 0):
                data = self.ser.read(self.ser.inWaiting())
                response = bytes_to_int32(data, self.is_big_endian)
                self.waiting_response = False
                return response
    
    def _send_value_and_read_response(self, value):
        self.send(value)
        return self.read_response()
    
    def send_cmd_and_data(self, cmd, data):
        self.send_value_and_read_response(cmd)
        self.send_value_and_read_response(data)
        return 0

    def request_data(self, cmd):
        data = self.send_value_and_read_response(cmd)
        return data