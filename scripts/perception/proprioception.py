#!/usr/bin/env python3

# from chardet import detect
import rospy
import sys
import cv2
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_perception_modules.yolo import InterbotixYoloInterface
from rdflib import Graph, Literal, RDF, URIRef
from rdflib.namespace import SOSA


class Proprioception():
    # Proprioception is the ability to feel the body's position, and location
    # This class monitors the robot's position
    def __init__(self):

        # Initialize node
        rospy.init_node("proprioception")

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        # Publishers
        self.workmem_topic = "perception_to_workmem"
        self.pub = rospy.Publisher(self.workmem_topic, String, queue_size=100)
        
        # Subscribers
        rospy.Subscriber("/locobot/joint_states", JointState, self.proprioceptionGraph)

        self.markers = {}
        self.image_in = ''
        self.debug_image = self.image_pub = rospy.Publisher('/smom/perception_debug', Image, queue_size=10)
        #self.
        
        # Initiate a graph for the sensory information
        self.perceptionGraph = Graph()
        self.camera = URIRef("http://example.org/locobot/camera")
        self.waist = URIRef("http://example.org/locobot/joints/waist")
        self.shoulder = URIRef("http://example.org/locobot/joints/shoulder")
        self.wrist_angle = URIRef("http://example.org/locobot/joints/wrist_angle")
        self.forearm_roll = URIRef("http://example.org/locobot/joints/forearm_roll")
        self.wrist_rotate = URIRef("http://example.org/locobot/joints/wrist_rotate")
        self.gripper = URIRef("http://example.org/locobot/joints/wrist_rotate")
        self.perceptionGraph.add((self.camera, RDF.type, SOSA.Sensor))
        
        # Buffers
        # self.motor_buffer = np.empty()
        # self.perc_buffer = np.empty()
        # self.decl_buffer = np.empty()
        # self.proc_buffer = np.empty()
        self.bridge = CvBridge()
        self.start()

    def proprioceptionGraph(self, msg):
        pass


    def start(self):
        while not rospy.is_shutdown():
            # Publish our custom message.
            #_, self.clusters = self.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
            #_, self.clusters = self.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
            # self.detections = self.yolo.getDetected()
            msg =  "this"
            rospy.logfatal(msg)
            self.pub.publish(msg)
            self.loop_rate.sleep()

    # def start(self):
    #     while not rospy.is_shutdown():
    #         # Publish our custom message.
    #         #_, self.clusters = self.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
    #         _, self.clusters = self.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
    #         self.detections = self.yolo.getDetected()
    #         msg =  self.constructPerception(self.clusters, self.detections)
    #         rospy.logerr(self.markers)
    #         self.pub.publish(msg)
    #         self.loop_rate.sleep()


def main(args):   
    try:
        #Perception()
        rospy.init_node("proprioception")
        rospy.logwarn('proprioception started')
        Proprioception()
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down perception node.")

if __name__ == '__main__':
    main(sys.argv)                      