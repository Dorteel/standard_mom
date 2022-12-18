#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from detection_msgs.msg import PerceivedObject, PerceivedObjects


class WorkingMemory():
    def __init__(self):
        pass

class ProceduralMemory():
    def __init__(self):
        self.query = None

    def getAffordance(self, obj):
        '''
        Constructs a query, looking for a property of an object
        Current use case checks if item has graspAffordance
        '''

        query = """
            PREFIX knowrob: <http://knowrob.org/kb/knowrob.owl#>
            ASK 
            {   ?object knowrob:hasAffordance knowrob:GraspingAffordance .
                ?object rdfs:label '""" + obj + """' .
            }"""
        return query

class Memories():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Initialize node
        rospy.init_node("memories")
        self.working_memory = WorkingMemory()
        self.procedural_memory = ProceduralMemory()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)
        self.objects = None
        # Messages types
        self.motor_msg = String
        self.proc_msg = String
        self.decl_msg = String
        self.perc_msg = String

        # Messages
        rospy.Subscriber("/standard_model/perception_output", PerceivedObjects, self.perception_cb)
        rospy.Subscriber("/declarative_memory/response", String, self.query_cb)
        self.query_pub = rospy.Publisher('/working_memory/query', String, queue_size=100)

        self.start()

    def query_cb(self, msg):
        self.query_response = msg.data
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
            # Publish our custom message.
            rospy.loginfo(self.objects)
            obj = 'cup'
            q = ProceduralMemory.getAffordance(obj, obj)
            self.query_pub.publish(q)
            self.loop_rate.sleep()


def main(args):   
    try:
        Memories()
        
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down procedural memory node.")

if __name__ == '__main__':
    main(sys.argv)                                       