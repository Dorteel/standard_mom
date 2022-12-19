#!/usr/bin/env python3

from __future__ import print_function

from standard_mom.srv import AnswerQuery
import rospy

def handle_answer_query(req):
    print("Query was [%s]"%(req.query))
    return req.query

def answer_query_server():
    rospy.init_node('answer_query_server')
    s = rospy.Service('answer_query', AnswerQuery, handle_answer_query)
    
    rospy.spin()

if __name__ == "__main__":
    answer_query_server()