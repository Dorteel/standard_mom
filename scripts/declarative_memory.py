#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np

class DeclarativeMemory():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Initialize node
        rospy.init_node("declarative memory")

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        # Messages types
        self.working_mem_msg = String

        # Publishers
        self.pub_topic = "declmem_to_workmem"
        self.pub = rospy.Publisher(self.pub_topic, self.working_mem_msg, queue_size=100)

        ## Subscribers
        self.from_wm = "workmem_to_declmem"
        rospy.Subscriber(self.from_wm, self.working_mem_msg, self.from_wm_cb)

        self.start()

    def from_wm_cb(self, msg):
        self.msg = msg
        #rospy.loginfo("Received from working memory: {}".format(self.msg.data))


    def start(self):
        while not rospy.is_shutdown():
            # Publish our custom message.
            dummy_msg = "Knowledge"
            self.pub.publish(dummy_msg)
            self.loop_rate.sleep()


def main(args):   
    try:
        DeclarativeMemory()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down declarative memory node.")

if __name__ == '__main__':
    main(sys.argv)                      