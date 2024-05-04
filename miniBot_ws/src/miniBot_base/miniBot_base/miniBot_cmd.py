"""
Class that contains all the commands that the miniBot can execute
"""
from serial_comm import MiniBotSerial
import time
# import rospy
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu

class MiniBotCmd():
    def __init__(self, retries = 5, port = "/dev/serial_sdk", baudrate = "921600"):
        self.miniBotSerial = MiniBotSerial(port, baudrate, retries)
        self.retries = retries
        self.port = port
        self.baudrate = baudrate
        self.cmd_registery()

    def cmd_registery(self):
        self.cmd_dict = {
            "__RESERVED__": 0x00000200,
            "CMD_VEL":      0x00000201,
            "ODOM":         0x00000202,
            "IMU":          0x00000203,
        }
        self.cmd_str = {v: k for k, v in self.cmd_dict.items()} # Inverse of cmd_dict e.g. {0: 'PACKET_STATUS', ...

    def send_cmd(self, cmd, data):
        cmd = self.cmd_str[cmd]
        for i in range(self.retries):
            self.miniBotSerial.write_packet(cmd, data)
            self.miniBotSerial.read_packet(cmd)
            if (self.miniBotSerial.read_success):
                break

    def set_cmd_vel(self, twist_msg):
        cmd = self.cmd_dict["CMD_VEL"]
        data = [twist_msg.linear.x, 
                twist_msg.linear.y, 
                twist_msg.linear.z, 
                twist_msg.angular.x, 
                twist_msg.angular.y, 
                twist_msg.angular.z]
        self.send_cmd(cmd, data)
        if (self.miniBotSerial.buffer == [cmd] + data):
            # print("Command sent successfully")
            return True
        
    def get_odom(self, odom_msg):
        cmd = self.cmd_dict["ODOM"]
        data = [0]
        self.send_cmd(cmd, data)
        buffer = self.miniBotSerial.buffer[1:]
        # odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = buffer[0]
        odom_msg.pose.pose.position.y = buffer[1]
        odom_msg.pose.pose.position.z = buffer[2]
        odom_msg.pose.pose.orientation.x = buffer[3]
        odom_msg.pose.pose.orientation.y = buffer[4]
        odom_msg.pose.pose.orientation.z = buffer[5]
        odom_msg.pose.pose.orientation.w = buffer[6]
        odom_msg.twist.twist.linear.x = buffer[7]
        odom_msg.twist.twist.linear.y = buffer[8]
        odom_msg.twist.twist.linear.z = buffer[9]
        odom_msg.twist.twist.angular.x = buffer[10]
        odom_msg.twist.twist.angular.y = buffer[11]
        odom_msg.twist.twist.angular.z = buffer[12]
        return odom_msg

    def get_imu(self, imu_msg):
        cmd = self.cmd_dict["IMU"]
        data = [0]
        self.send_cmd(cmd, data)
        buffer = self.miniBotSerial.buffer[1:]
        # imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu"
        imu_msg.orientation.x = buffer[0]
        imu_msg.orientation.y = buffer[1]
        imu_msg.orientation.z = buffer[2]
        imu_msg.orientation.w = buffer[3]
        imu_msg.angular_velocity.x = buffer[4]
        imu_msg.angular_velocity.y = buffer[5]
        imu_msg.angular_velocity.z = buffer[6]
        imu_msg.linear_acceleration.x = buffer[7]
        imu_msg.linear_acceleration.y = buffer[8]
        imu_msg.linear_acceleration.z = buffer[9]
        return imu_msg


    