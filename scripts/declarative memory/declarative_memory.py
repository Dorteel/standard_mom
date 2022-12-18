#!/usr/bin/env python3

import rospy
import sys

from std_msgs.msg import String

from rdflib.plugins.sparql import prepareQuery
from rdflib import Graph, URIRef, Namespace, Literal
from rdflib.namespace import RDFS, XSD



class DeclarativeMemory():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        ##TODO: Change the ros messages into a service
        # Initialize node
        rospy.init_node("declarative_memory")
        self.loop_rate = rospy.Rate(100)

        self.kg = Graph().parse('memory.ttl')
        self.response = rospy.Publisher('/declarative_memory/response', String, queue_size=100)
        rospy.Subscriber('working_memory/query', String, self.query_cb)

        self.start()

    def query_cb(self, query):
        answer = [item for item in self.kg.query(query.data)][0][0]
        self.response.publish(answer)

    def start(self):
        while not rospy.is_shutdown():
            self.loop_rate.sleep()


def main(args):   
    try:
        DeclarativeMemory()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down declarative memory node.")

if __name__ == '__main__':
    main(sys.argv)                      