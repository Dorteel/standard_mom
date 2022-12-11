#!/usr/bin/env python3

# from chardet import detect
from sklearn.neighbors import LocalOutlierFactor
import rospy
import sys
import cv2
import os
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from std_msgs.msg import String
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_perception_modules.yolo import InterbotixYoloInterface
from rdflib import Graph, Literal, RDF, URIRef, Namespace
from rdflib.namespace import SOSA


class Vision():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Initialize node
        rospy.init_node("perception")
        self.pcl = InterbotixPointCloudInterface("locobot" + "/pc_filter", False)
        self.yolo = InterbotixYoloInterface()
        #_, self.clusters = self.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)

        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)

        # Messages types
        self.locobot_msg = Image
        self.work_msg = String
        self.perc_msg = String

        # Publishers
        self.workmem_topic = "perception_to_workmem"
        self.pub = rospy.Publisher(self.workmem_topic, self.perc_msg, queue_size=100)
        
        # Subscribers
        self.work_listen_topic = "workmem_to_perception"
        rospy.Subscriber(self.work_listen_topic, self.work_msg, self.from_wm_cb)

        self.locobot_topic = "/locobot/camera/color/image_raw"
        rospy.Subscriber(self.locobot_topic, self.locobot_msg, self.from_wm_cb)

        # Debugging the image
        rospy.Subscriber('/yolov5/image_out', Image, self.image_callback, queue_size=1)
        rospy.Subscriber('/locobot/pc_filter/markers/objects', Marker, self.marker_callback, queue_size=1)
        self.markers = {}
        self.image_in = ''
        self.debug_image = self.image_pub = rospy.Publisher('/smom/perception_debug', Image, queue_size=10)
        #self.
        
        # Initiate a graph for the sensory information
        self.locobot = Namespace("http://example.org/locobot/")
        self.initSceneGraph()
        # Buffers
        # self.motor_buffer = np.empty()
        # self.perc_buffer = np.empty()
        # self.decl_buffer = np.empty()
        # self.proc_buffer = np.empty()
        self.bridge = CvBridge()
        self.start()

    def from_wm_cb(self, msg):
        self.msg = msg
        #rospy.loginfo("{}: Received from working memory: {}".format("Perception", self.msg.data))

    def image_callback(self, data):
        self.image_in = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
    def marker_callback(self, marker):
        self.markers[marker.id] = marker.pose.position

    def constructPerception(self, clusters, detections):
        rospy.logwarn("Clusters: {}".format(clusters))
        rospy.logwarn("Detection: {}".format(detections))
        rospy.logerr("----------")
        return "True"

    def initSceneGraph(self):
        # Using random nodes for the properties until I find a proper ontology
        self.perceptionGraph = Graph()
        self.perceptionGraph.add((self.locobot.camera, RDF.type, SOSA.Sensor))
        self.color = URIRef("http://example.org/Color/")
        self.location = URIRef("http://example.org/Location/")
        self.numberofpoints = URIRef("http://example.org/NumberOfPoints/")
        self.boundingbox = URIRef("http://example.org/BoundingBox/")
        self.perceptionGraph.add((self.color, RDF.type, SOSA.ObservableProperty))
        self.perceptionGraph.add((self.location, RDF.type, SOSA.ObservableProperty))
        self.perceptionGraph.add((self.numberofpoints, RDF.type, SOSA.ObservableProperty))
        self.perceptionGraph.add((self.boundingbox, RDF.type, SOSA.ObservableProperty))


    def generateSceneGraph(self):
        """
        Scene graphs are generated from the detection and returned in a graph
        """
    
        
        for cluser in self.clusters:
            observation = URIRef("http://example.org/Observation/" + cluser['name'])
            self.perceptionGraph.add((observation, RDF.type, SOSA.Observation))
            self.perceptionGraph.add((Literal(str(cluser['position'])), RDF.type, self.location))
            self.perceptionGraph.add((Literal(str(cluser['color'])), RDF.type, self.color))
            self.perceptionGraph.add((Literal(str(cluser['num_points'])), RDF.type, self.numberofpoints))
        rospy.logerr(len(self.perceptionGraph))
        # TODO: Put generated detections into a scene graph
        #for key, value in detections:
        #    sceneGraph.add()
        
        return None


    def start(self):
        while not rospy.is_shutdown():
            # Publish our custom message.
            #_, self.clusters = self.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
            _, self.clusters = self.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
            #self.detections = self.yolo.getDetected()
            self.generateSceneGraph()
            rospy.logerr(self.markers)
            self.saveKG("test.ttl")
            #self.pub.publish(msg)
            self.loop_rate.sleep()

    def saveKG(self, name):
         self.perceptionGraph.serialize(destination="/home/user/locobot_ws/src/standard_model_of_mind/standard_mom/scripts/perception/" + name)


def main(args):   
    try:
        Vision()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down perception node.")

if __name__ == '__main__':
    main(sys.argv)                      