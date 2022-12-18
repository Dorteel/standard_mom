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

        self.objects = None

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
        

        self.perc_listen_topic = "perception_to_workmem"
        rospy.Subscriber(self.perc_listen_topic, self.perc_msg, self.perception_cb)

        self.proc_listen_topic = "procmem_to_workmem"
        rospy.Subscriber(self.proc_listen_topic, self.proc_msg, self.procedural_cb)

        # Buffers
        # self.motor_buffer = np.empty()
        # self.perc_buffer = np.empty()
        # self.decl_buffer = np.empty()
        # self.proc_buffer = np.empty()

        self.start()


    def perception_cb(self, msg):
        '''
        Receives the objects from the perception module and generates the scene graph
        '''
        self.objects = msg.detected_objects
        #rospy.loginfo(self.objects)

    def declarative_cb(self, msg):
        self.msg = msg
        #rospy.logdebug("Received from declarative memory: {}".format(self.msg.data))

    def procedural_cb(self, msg):
        self.msg = msg
        #rospy.loginfo("Received from procedural memory: {}".format(self.msg.data))


    def start(self):
        while not rospy.is_shutdown():
            # Perception loop
            rospy.loginfo(self.objects)
            # Publish our custom message.
            # rospy.loginfo('Running..')
            # dummy_msg = "Move ahead"
            # self.motor_pub.publish(dummy_msg)
            # dummy_msg = "I saw this..."
            # self.decl_pub.publish(dummy_msg)
            # dummy_msg = "Action required"
            # self.proc_pub.publish(dummy_msg)
            # dummy_msg = "Msg 4 perception"
            # self.perc_pub.publish(dummy_msg)
            rospy.loginfo(self.objects)
            self.loop_rate.sleep()
            



def main(args):   
    try:
        WorkingMemory()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down working memory node.")

if __name__ == '__main__':
    main(sys.argv)                      