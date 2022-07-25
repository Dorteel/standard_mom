#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

class Perception():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Initialize node
        rospy.init_node("procedural memory")

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        # Messages types
        self.work_msg = String
        self.perc_msg = String


        # Publishers
        self.workmem_topic = "perception_to_workmem"
        self.pub = rospy.Publisher(self.workmem_topic, self.work_msg, queue_size=100)

        ## Subscribers
        self.work_listen_topic = "workmem_to_perception"
        rospy.Subscriber(self.work_listen_topic, self.work_msg, self.from_wm_cb)

        # Buffers
        # self.motor_buffer = np.empty()
        # self.perc_buffer = np.empty()
        # self.decl_buffer = np.empty()
        # self.proc_buffer = np.empty()

        self.start()

    def from_wm_cb(self, msg):
        self.msg = msg
        rospy.loginfo("{}: Received from working memory: {}".format("Perception", self.msg.data))


    def start(self):
        while not rospy.is_shutdown():
            # Publish our custom message.
            self.loop_rate.sleep()


def main(args):   
    try:
        Perception()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down perception node.")

if __name__ == '__main__':
    main(sys.argv)                      