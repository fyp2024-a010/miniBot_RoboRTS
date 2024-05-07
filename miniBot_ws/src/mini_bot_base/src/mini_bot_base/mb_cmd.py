import time

from mb_list import MiniBotCMDList
from mb_serial import MiniBotSerial

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class MiniBotCmd():
    def __init__(self, port = "/dev/serial_sdk", baudrate = "921600", retries = 10, is_big_endian = 0):
        self.communicate = MiniBotSerial(port, baudrate, retries, is_big_endian)
        self.cmd_dict = MiniBotCMDList().get_cmd_dictionary()
        self.significant_figure = 1e5
    
    def set_cmd_vel(self, twist_msg):
        cmd = self.cmd_dict["CMD_VEL_LINEAR_X"]
        data = int(twist_msg.linear.x * self.significant_figure)
        self.communicate.send_cmd_and_data(cmd, data)
        cmd = self.cmd_dict["CMD_VEL_LINEAR_Y"]
        data = int(twist_msg.linear.y * self.significant_figure)
        self.communicate.send_cmd_and_data(cmd, data)
        cmd = self.cmd_dict["CMD_VEL_LINEAR_Z"]
        data = int(twist_msg.linear.z * self.significant_figure)
        self.communicate.send_cmd_and_data(cmd, data)
        cmd = self.cmd_dict["CMD_VEL_ANGULAR_X"]
        data = int(twist_msg.angular.x * self.significant_figure)
        self.communicate.send_cmd_and_data(cmd, data)
        cmd = self.cmd_dict["CMD_VEL_ANGULAR_Y"]
        data = int(twist_msg.angular.y * self.significant_figure)
        self.communicate.send_cmd_and_data(cmd, data)
        cmd = self.cmd_dict["CMD_VEL_ANGULAR_Z"]
        data = int(twist_msg.angular.z * self.significant_figure)
        self.communicate.send_cmd_and_data(cmd, data)
        return twist_msg
    
    def get_odom(self, odom_msg):
        cmd = self.cmd_dict["ODOM_POINT_X"]
        odom_msg.pose.pose.position.x = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_POINT_Y"]
        odom_msg.pose.pose.position.y = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_POINT_Z"]
        odom_msg.pose.pose.position.z = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_QUATERNION_X"]
        odom_msg.pose.pose.orientation.x = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_QUATERNION_Y"]
        odom_msg.pose.pose.orientation.y = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_QUATERNION_Z"]
        odom_msg.pose.pose.orientation.z = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_QUATERNION_W"]
        odom_msg.pose.pose.orientation.w = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_LINEAR_X"]
        odom_msg.twist.twist.linear.x = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_LINEAR_Y"]
        odom_msg.twist.twist.linear.y = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_LINEAR_Z"]
        odom_msg.twist.twist.linear.z = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_ANGULAR_X"]
        odom_msg.twist.twist.angular.x = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_ANGULAR_Y"]
        odom_msg.twist.twist.angular.y = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["ODOM_ANGULAR_Z"]
        odom_msg.twist.twist.angular.z = self.communicate.request_data(cmd) / self.significant_figure
        return odom_msg
    
    def get_imu(self, imu_msg):
        cmd = self.cmd_dict["IMU_QUATERNION_X"]
        imu_msg.orientation.x = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["IMU_QUATERNION_Y"]
        imu_msg.orientation.y = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["IMU_QUATERNION_Z"]
        imu_msg.orientation.z = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["IMU_QUATERNION_W"]
        imu_msg.orientation.w = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["IMU_ANGULAR_X"]
        imu_msg.angular_velocity.x = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["IMU_ANGULAR_Y"]
        imu_msg.angular_velocity.y = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["IMU_ANGULAR_Z"]
        imu_msg.angular_velocity.z = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["IMU_ACCEL_X"]
        imu_msg.linear_acceleration.x = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["IMU_ACCEL_Y"]
        imu_msg.linear_acceleration.y = self.communicate.request_data(cmd) / self.significant_figure
        cmd = self.cmd_dict["IMU_ACCEL_Z"]
        imu_msg.linear_acceleration.z = self.communicate.request_data(cmd) / self.significant_figure
        return imu_msg

def main():
    mini_bot_cmd = MiniBotCmd()
    if (mini_bot_cmd.communicate.ser.is_open):
        try:
            while True:
                cmd = 0x201
                data = 20000
                # mini_bot_cmd.communicate.send_cmd_and_data(cmd, data)
                data = mini_bot_cmd.communicate.request_data()
                print("Data: ", data)
        except KeyboardInterrupt:
            print("Keyboard interrupt")
            print("Closing serial port...")
        finally:
            mini_bot_cmd.communicate.close_port()

if __name__ == "__main__":
    main()
