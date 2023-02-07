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
        rospy.loginfo('Memory modules initialized')
        # Messages
        self.action_pub = rospy.Publisher('workmem_to_motors', PerceivedObject,queue_size=20)
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
            if self.objects:
                for obj in self.objects:
                    if not obj.label:
                        rospy.loginfo("Identity of {} is not recognised".format(obj.name))
                        continue
                    objType = obj.label
                    q = self.procedural_memory.getAffordance(objType)
                    response = self.working_memory.query_dm_client(q)
                    if response == 'True':
                        #rospy.logerr(response)
                        rospy.logerr("{} is a GraspableObject".format(objType))
                        self.action_pub.publish(obj)
                    else:
                        rospy.logerr("{} is not a GraspableObject".format(objType))
            self.loop_rate.sleep()

def main(args):   
    try:
        Memories()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down memory node.")

if __name__ == '__main__':
    main(sys.argv)                                       