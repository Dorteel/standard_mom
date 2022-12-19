#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from detection_msgs.msg import PerceivedObject, PerceivedObjects
from standard_mom.srv import *

class WorkingMemory():
    def __init__(self):
        pass

    def query_dm_client(self, query):
        rospy.wait_for_service('answer_query')
        try:
            query_dm = rospy.ServiceProxy('answer_query', AnswerQuery)
            resp1 = query_dm(query)
            return resp1.response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


class ProceduralMemory():
    def __init__(self):
        self.query = None

    def getAffordance(self, obj):
        '''
        Constructs a query, looking for a property of an object
        Current use case checks if item has graspAffordance
        '''
        test_q = """
        PREFIX knowrob: <http://knowrob.org/kb/knowrob.owl#>
        SELECT ?label WHERE 
        {   ?object knowrob:hasAffordance knowrob:GraspingAffordance .
            ?object rdfs:label ?label .
        }
        """

        query = """
            PREFIX knowrob: <http://knowrob.org/kb/knowrob.owl#>
            ASK 
            {   ?object knowrob:hasAffordance knowrob:GraspingAffordance .
                ?object rdfs:label '""" + obj + """' .
            }"""
        return test_q

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

        # Messages
        rospy.Subscriber("/standard_model/perception_output", PerceivedObjects, self.perception_cb)
        self.start()


    def perception_cb(self, msg):
        '''
        Receives the objects from the perception module and generates the scene graph
        '''
        self.objects = msg.detected_objects


    def start(self):
        while not rospy.is_shutdown():
            # Publish our custom message.
            #rospy.loginfo(self.objects)
            obj = 'cup'
            q = self.procedural_memory.getAffordance(obj)
            response = self.working_memory.query_dm_client(q)
            print(response)
            #self.query_pub.publish(q)
            self.loop_rate.sleep()


def main(args):   
    try:
        Memories()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down memory node.")

if __name__ == '__main__':
    main(sys.argv)                                       