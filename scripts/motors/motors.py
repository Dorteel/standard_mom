#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String
import numpy as np
from interbotix_xs_modules.locobot import InterbotixLocobotXS

class Motors():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Initialize node
        rospy.init_node("motors")
        self.bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s", init_node=False)
        self.counter = 0
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        # Messages types
        self.work_msg = String
        # self.proc_msg = String


        # Publishers
        # self.workmem_topic = "procmem_to_workmem"
        # self.pub = rospy.Publisher(self.workmem_topic, self.proc_msg, queue_size=100)

        ## Subscribers
        self.work_listen_topic = "workmem_to_motors"
        rospy.Subscriber(self.work_listen_topic, self.work_msg, self.from_wm_cb)

        # Buffers
        # self.motor_buffer = np.empty()
        # self.perc_buffer = np.empty()
        # self.decl_buffer = np.empty()
        # self.proc_buffer = np.empty()

        self.start()

    def from_wm_cb(self, msg):
        self.msg = msg
        #rospy.loginfo("Received from working memory: {}".format(self.msg.data))


    def start(self):
        while not rospy.is_shutdown():
            if self.counter <= 2:
                self.bot.camera.pan_tilt_move(0.4, 0.9)
                self.bot.arm.set_ee_pose_components(x=0.3, z=0.2, moving_time=1.5)
                self.bot.gripper.open()
                rospy.loginfo("Step: {}".format(self.counter))
            else:
                self.bot.arm.go_to_sleep_pose()
            self.counter += 1
            self.loop_rate.sleep()


def main(args):   
    try:
        Motors()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down motor node.")

if __name__ == '__main__':
    main(sys.argv)                      