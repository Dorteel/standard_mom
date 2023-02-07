#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from standard_mom.srv import *

def answer_query_client(query):
    rospy.wait_for_service('answer_query')
    try:
        answer_query = rospy.ServiceProxy('answer_query', AnswerQuery)
        resp1 = answer_query(query)
        return resp1.response
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    q = """
    PREFIX knowrob: <http://knowrob.org/kb/knowrob.owl#>
    SELECT ?label WHERE 
    {   ?object knowrob:hasAffordance knowrob:GraspingAffordance .
        ?object rdfs:label ?label .
    }"""
    print("Requesting %s"%(q))
    print("%s"%(answer_query_client(q)))