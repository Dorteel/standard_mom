#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String
import numpy as np

class Motors():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Initialize node
        rospy.init_node("motors")

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
        rospy.loginfo("Received from working memory: {}".format(self.msg.data))


    def start(self):
        while not rospy.is_shutdown():

            self.loop_rate.sleep()


def main(args):   
    try:
        Motors()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down motor node.")

if __name__ == '__main__':
    main(sys.argv)                      