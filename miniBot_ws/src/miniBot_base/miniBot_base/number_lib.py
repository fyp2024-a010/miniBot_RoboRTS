"""
Library for number to byte array conversion and vice versa
"""
import struct

"""
Basic conversion functions
"""

def bytes_to_int32(data, is_big_endian = 0):
    if (len(data) != 4):
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

"""
Custom conversion functions
"""

def mb_int_to_float(int_value, accuracy = 1e5):
    return int_value/accuracy

def mb_float_to_int(float_value, accuracy = 1e5):
    return int(float_value*accuracy)

