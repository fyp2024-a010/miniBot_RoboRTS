class MiniBotCMDList:
    def __init__(self):
        self.cmd_dictionary = self.get_cmd_dictionary()

    def get_cmd_dictionary(self):
        cmd_dict = {"__RESERVED__" :        0x100, 
                    "ACK" :                 0x101, 
                    "IGNORE" :              0x102,
                    # CMD_VEL
                    "CMD_VEL" :             0x200,
                    "CMD_VEL_LINEAR_X" :    0x201,
                    "CMD_VEL_LINEAR_Y" :    0x202,
                    "CMD_VEL_LINEAR_Z" :    0x203,
                    "CMD_VEL_ANGULAR_X" :   0x204,
                    "CMD_VEL_ANGULAR_Y" :   0x205,
                    "CMD_VEL_ANGULAR_Z" :   0x206,
                    # ODOM
                    "ODOM":			        0x300,
                    "ODOM_POINT_X":	        0x301,
                    "ODOM_POINT_Y":	        0x302,
                    "ODOM_POINT_Z":	        0x303,
                    "ODOM_QUATERNION_X":	0x304,
                    "ODOM_QUATERNION_Y":	0x305,
                    "ODOM_QUATERNION_Z":	0x306,
                    "ODOM_QUATERNION_W":	0x307,
                    "ODOM_LINEAR_X":	    0x308,
                    "ODOM_LINEAR_Y":	    0x309,
                    "ODOM_LINEAR_Z":	    0x310,
                    "ODOM_ANGULAR_X":	    0x311,
                    "ODOM_ANGULAR_Y":	    0x312,
                    "ODOM_ANGULAR_Z":	    0x313,
                    # IMU
                    "IMU":			        0x400,
                    "IMU_QUATERNION_X":	    0x401,
                    "IMU_QUATERNION_Y":	    0x402,
                    "IMU_QUATERNION_Z":	    0x403,
                    "IMU_QUATERNION_W":	    0x404,
                    "IMU_ANGULAR_X":	    0x405,
                    "IMU_ANGULAR_Y":	    0x406,
                    "IMU_ANGULAR_Z":	    0x407,
                    "IMU_ACCEL_X":	        0x408,
                    "IMU_ACCEL_Y":	        0x409,
                    "IMU_ACCEL_Z":	        0x410
        }
        return cmd_dict