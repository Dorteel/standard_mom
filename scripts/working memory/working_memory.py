#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from detection_msgs.msg import PerceivedObject, PerceivedObjects

class WorkingMemory():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Initialize node
        rospy.init_node("working_memory")

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        # Messages types
        self.motor_msg = String
        self.proc_msg = String
        self.decl_msg = String
        self.perc_msg = String

        # Publishers
        self.motor_topic = "workmem_to_motors"
        self.motor_pub = rospy.Publisher(self.motor_topic, self.motor_msg, queue_size=100)

        self.proc_talk_topic = "workmem_to_procmem"
        self.proc_pub = rospy.Publisher(self.proc_talk_topic, self.proc_msg, queue_size=100)
        
        self.decl_talk_topic = "workmem_to_declmem"
        self.decl_pub = rospy.Publisher(self.decl_talk_topic, self.decl_msg, queue_size=100)

        self.perc_talk_topic = "workmem_to_perception"
        self.perc_pub = rospy.Publisher(self.perc_talk_topic, self.perc_msg, queue_size=100)

        ## Subscribers
        self.decl_listen_topic = "declmem_to_workmem"
        rospy.Subscriber(self.decl_listen_topic, self.decl_msg, self.declarative_cb)

        rospy.Subscriber("/standard_model/perception_output", PerceivedObjects, self.perception_cb)

        self.proc_listen_topic = "procmem_to_workmem"
        rospy.Subscriber(self.proc_listen_topic, self.proc_msg, self.procedural_cb)

        self.start()

    def declarative_cb(self, msg):
        self.msg = msg
        #rospy.logdebug("Received from declarative memory: {}".format(self.msg.data))

    def perception_cb(self, msg):
        '''
        Receives the objects from the perception module and generates the scene graph
        '''
        self.objects = msg.detected_objects
        #rospy.loginfo(self.objects)

    def procedural_cb(self, msg):
        self.msg = msg
        #rospy.loginfo("Received from procedural memory: {}".format(self.msg.data))


    def start(self):
        while not rospy.is_shutdown():
            
            self.loop_rate.sleep()


class ProceduralMemory():
    def __init__(self):

        # Initialize node
        rospy.init_node("procedural_memory")

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        # Messages types
        self.work_msg = String
        self.proc_msg = String


        # Publishers
        self.workmem_topic = "procmem_to_workmem"
        self.pub = rospy.Publisher(self.workmem_topic, self.proc_msg, queue_size=100)

        ## Subscribers
        self.work_listen_topic = "workmem_to_procmem"
        rospy.Subscriber(self.work_listen_topic, self.work_msg, self.from_wm_cb)

        # Buffers
        # self.motor_buffer = np.empty()
        # self.perc_buffer = np.empty()
        # self.decl_buffer = np.empty()
        # self.proc_buffer = np.empty()
        self.workingMemory = WorkingMemory()
        WorkingMemory.start()
        self.start()

    def from_wm_cb(self, msg):
        self.msg = msg
        #rospy.loginfo("Received from working memory: {}".format(self.msg.data))


    def start(self):
        while not rospy.is_shutdown():
            # Publish our custom message.
            dummy_msg = "Move ahead"
            self.pub.publish(dummy_msg)
            rospy.loginfo('Procedural memory runnning')
            self.loop_rate.sleep()


def main(args):   
    try:
        ProceduralMemory()
        
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down procedural memory node.")

if __name__ == '__main__':
    main(sys.argv)                                       