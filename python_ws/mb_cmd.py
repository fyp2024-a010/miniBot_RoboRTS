import time

from mb_cmd_list import MiniBotCMDList
from mb_serial import MiniBotSerial

class MiniBotCmd():
    def __init__(self, port = "/dev/serial_sdk", baudrate = "921600", retries = 10, is_big_endian = 0):
        self.communicate = MiniBotSerial(port, baudrate, retries, is_big_endian)
        self.cmd_dict = MiniBotCMDList().get_cmd_dictionary()

def main():
    mini_bot_cmd = MiniBotCmd()
    if (mini_bot_cmd.communicate.ser.is_open):
        try:
            while True:
                cmd = mini_bot_cmd.cmd_dict["CMD_VEL_LINEAR_X"]
                data = 20000
                mini_bot_cmd.communicate.send_cmd_and_data(cmd, data)
                # data = mini_bot_cmd.communicate.request_data(cmd)
                # print("Data: ", data)
                # time.sleep(0.0001)
        except KeyboardInterrupt:
            print("Keyboard interrupt")
            print("Closing serial port...")
        finally:
            mini_bot_cmd.communicate.close_port()

if __name__ == "__main__":
    main()
