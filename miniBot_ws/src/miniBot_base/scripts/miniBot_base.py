import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from miniBot_base.miniBot_cmd import MiniBotCmd

class miniBotBaseNode():
    def __init__(self):
        rospy.init_node("miniBotBaseNode", anonymous=False)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=1)
        self.miniBotCmd = MiniBotCmd()
        self.odom_msg = Odometry()
        self.imu_msg = Imu()
    
    def cmd_vel_callback(self, twist_msg):
        self.miniBotCmd.set_cmd_vel(twist_msg)

    def get_odom(self):
        self.miniBotCmd.get_odom(self.odom_msg)
        self.odom_pub.publish(self.odom_msg)

    def get_imu(self):
        self.miniBotCmd.get_imu(self.imu_msg)
        self.imu_pub.publish(self.imu_msg)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.get_odom()
            self.get_imu()
            rate.sleep()

def main():
    mini_bot_base_node = miniBotBaseNode()
    try:
        mini_bot_base_node.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
