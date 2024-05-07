#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from mini_bot_base.mb_cmd import MiniBotCmd
# from mb_cmd import MiniBotCmd

def cmd_vel_callback(msg):
    global imu_msg, odom_msg, twist_msg
    twist_msg = msg

def state_update_timer_callback(event):
    global imu_msg, odom_msg, twist_msg
    mini_bot_cmd.set_cmd_vel(twist_msg)
    imu_msg = mini_bot_cmd.get_imu(imu_msg)
    odom_msg = mini_bot_cmd.get_odom(odom_msg)

def publish_timer_callback(event):
    global imu_msg, odom_msg, twist_msg
    imu_publisher.publish(imu_msg)
    odom_publisher.publish(odom_msg)

def mini_bot_cmd_node(hz = 50):
    global mini_bot_cmd, imu_publisher, odom_publisher, cmd_vel_subscriber
    global imu_msg, odom_msg, twist_msg
    imu_msg = Imu()
    odom_msg = Odometry()
    twist_msg = Twist()

    mini_bot_cmd = MiniBotCmd(port = "/dev/ttyACM0", baudrate = "921600", retries = 10, is_big_endian = 0)

    rospy.init_node('miniBotBaseNode', anonymous=False)

    publish_timer = rospy.Timer(rospy.Duration(1.0/hz), publish_timer_callback)
    state_update_timer = rospy.Timer(rospy.Duration(1.0/2.0/hz), state_update_timer_callback)

    imu_publisher = rospy.Publisher('/imu', Imu, queue_size=1)
    odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
    cmd_vel_subscriber = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()
    state_update_timer.shutdown()
    publish_timer.shutdown()
        
    # rate = rospy.Rate(hz)
    # while not rospy.is_shutdown():
    #     mini_bot_cmd.set_cmd_vel(mb_twist_msg)
    #     imu_msg = mini_bot_cmd.get_imu(imu_msg)
    #     odom_msg = mini_bot_cmd.get_odom(odom_msg)
    #     imu_publisher.publish(imu_msg)
    #     odom_publisher.publish(odom_msg)
    #     rate.sleep()

def main():
    mini_bot_cmd_node()

if __name__ == "__main__":
    main()
