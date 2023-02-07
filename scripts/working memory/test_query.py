#!/usr/bin/env python3

import rospy
import sys

from std_msgs.msg import String

from rdflib.plugins.sparql import prepareQuery
from rdflib import Graph, URIRef, Namespace, Literal
from rdflib.namespace import RDFS, XSD



class Querier():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Initialize node
        rospy.init_node("test_memory")
        self.loop_rate = rospy.Rate(100)


        self.response = rospy.Publisher('/working_memory/query', String, queue_size=100)

        self.start()

    def start(self):
        while not rospy.is_shutdown():
            query = """
            PREFIX knowrob: <http://knowrob.org/kb/knowrob.owl#>
            SELECT ?label WHERE 
            {   ?object knowrob:hasAffordance knowrob:GraspingAffordance .
                ?object rdfs:label ?label .
            }"""
            self.response.publish(query)
            self.loop_rate.sleep()


def main(args):   
    try:
        Querier()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down declarative memory node.")

if __name__ == '__main__':
    main(sys.argv)                      