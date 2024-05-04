#!/usr/bin/env python

import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# from miniBot_base.miniBot_cmd import MiniBotCmd
from mb_cmd import MiniBotCmd

class miniBotBaseNode():
    def __init__(self):
        rospy.init_node("miniBotBaseNode", anonymous=False)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)
        self.miniBotCmd = MiniBotCmd(port = "/dev/serial_sdk", baudrate = "921600", retries = 10, is_big_endian = 0)
        self.odom_msg = Odometry()
        self.imu_msg = Imu()
    
    def cmd_vel_callback(self, twist_msg):
        self.miniBotCmd.set_cmd_vel(twist_msg)

    def get_odom(self):
        self.odom_msg = self.miniBotCmd.get_odom(self.odom_msg)
        self.odom_pub.publish(self.odom_msg)

    def get_imu(self):
        self.imu_msg = self.miniBotCmd.get_imu(self.imu_msg)
        self.imu_pub.publish(self.imu_msg)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.get_odom()
            self.get_imu()
            rate.sleep()

def startup_node_class():
    mini_bot_base_node = miniBotBaseNode()
    try:
        mini_bot_base_node.run()
    except rospy.ROSInterruptException:
        pass

def cmd_vel_callback(twist_msg):
    mini_bot_cmd.set_cmd_vel(twist_msg)

def node_subscribers():
    cmd_vel_subscriber
    rospy.spin()

def node_publishers(hz = 50):
    imu_msg = Imu()
    odom_msg = Odometry()
    rate = rospy.Rate(hz)
    while not rospy.is_shutdown():
        imu_msg = mini_bot_cmd.get_imu(imu_msg)
        odom_msg = mini_bot_cmd.get_odom(odom_msg)
        imu_publisher.publish(imu_msg)
        odom_publisher.publish(odom_msg)
        rate.sleep()

def mini_bot_cmd_node(hz = 50):
    global mini_bot_cmd, imu_publisher, odom_publisher, cmd_vel_subscriber

    mini_bot_cmd = MiniBotCmd(port = "/dev/serial_sdk", baudrate = "921600", retries = 10, is_big_endian = 0)

    rospy.init_node('miniBotBaseNode', anonymous=False)

    imu_publisher = rospy.Publisher('/imu', Imu, queue_size=1)
    odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
    cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
        
    node_subscribers()
    node_publishers()
    # threading.start_new_thread(node_subscribers, ())
    # threading.start_new_thread(node_publishers, (hz))

def main():
    mini_bot_cmd_node()
    # startup_node_class()

if __name__ == "__main__":
    main()
