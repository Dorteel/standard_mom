#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from interbotix_perception_modules.yolo import InterbotixYoloInterface
from interbotix_common_modules import angle_manipulation as ang
from rdflib import Graph, Literal, RDF, URIRef, Namespace
from rdflib.namespace import SOSA
from detection_msgs.msg import PerceivedObject, PerceivedObjects
from tf.transformations import euler_from_quaternion
import time
import sys
import tf2_ros


class VisionNode(object):
    def __init__(self):
        # Params
        self.bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")
        self.yolo = InterbotixYoloInterface()
        self.image = None
        self.clusters = []
        self.detections = []
        self.br = CvBridge()
        self.objects = []
        self.loop_rate = rospy.Rate(100)
        
        self.cam_frame =  'locobot/camera_color_optical_frame'
        self.arm_frame =  'locobot/arm_base_link'

        self.visionNS = URIRef("http://example.org/VisionProperies/")
        # Initialize prediction publisher
        self.detect_pub = rospy.Publisher("/standard_model/perception_output", PerceivedObjects, queue_size=10)
        self.pub = rospy.Publisher('test_image', Image,queue_size=20)
        # Subscribers
        #rospy.Subscriber("/locobot/camera/color/image_raw",Image,self.imgCallback)

        self.start()

    #def imgCallback(self, msg):
    #    self.image = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    # def show_clusters(self, bot, rf):
    #     _, self.clusters = bot.pcl.get_cluster_positions(ref_frame=rf, sort_axis="y", reverse=True)
    #     _, test_clusters = bot.pcl.get_cluster_positions(ref_frame=self.arm_frame, sort_axis="y", reverse=True)
    #     # for cluster in self.clusters:
    #     #     pixels = self.get_pixel_from3d(cluster['position'])
    #     #     # Draw on image the pixel coordinates
    #     #     self.image = cv2.circle(self.image, pixels, 10, (255, 0, 0), 3)


    def transform_frame(self, c, frf, tof):
        '''
        Transforms coordinates c from frame frf to frame tof
        '''
        # _, test_clusters = bot.pcl.get_cluster_positions(ref_frame=self.arm_frame, sort_axis="y", reverse=True)
        # test_clusters['position']
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        try:
            trans = tfBuffer.lookup_transform(frf, tof, rospy.Time(0), rospy.Duration(4.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("Failed to look up the transform from '%s' to '%s'." % (frf, tof))
            return False, []
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        z = trans.transform.translation.z
        quat = trans.transform.rotation
        q = [quat.x, quat.y, quat.z, quat.w]
        rpy = euler_from_quaternion(q)
        T_rc = ang.poseToTransformationMatrix([x, y, z, rpy[0], rpy[1], rpy[2]])
        p_co = [c.position.x, c.position.y, c.position.z, 1]
        p_ro = np.dot(T_rc, p_co)
        # p_comin = [c.min_z_point.x, c.min_z_point.y, c.min_z_point.z, 1]
        # p_romin = np.dot(T_rc, p_comin)
        # p_ro[2] = p_romin[2]
        return p_ro[:3]

    def get_pixel_from3d(self, coords):
        ''''
        Takes as input 3D coordinates, outputs two pixels
        '''
        coords = np.append(coords, 1)
        # Intrinsic matrix - hardcoded for now, TODO: Change it to subscription
        P = np.array([380.7534484863281, 0.0, 318.1063232421875, 0.0, 0.0, 380.7534484863281, 235.16383361816406, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32).reshape(3,4)
        pix = np.matmul(P, coords)
        c = np.array([pix[0]/pix[2], pix[1]/pix[2]]).astype('int32')
        return c

    def get_objects(self):
        '''
        Looks through the detected clusters and matches them with the detected bounding boxes
        '''
        _, self.clusters = self.bot.pcl.get_cluster_positions(ref_frame=self.cam_frame, sort_axis="y", reverse=True)
        _, test_clusters = self.bot.pcl.get_cluster_positions(ref_frame=self.arm_frame, sort_axis="y", reverse=True)
        objects = PerceivedObjects()
        objects.header.stamp = rospy.Time.now()
        for c in self.clusters:
            arm_clusters = [cluster for cluster in test_clusters if cluster['name'] == c['name']]
            if len(arm_clusters): arm_c = arm_clusters[0]
            else: continue
            object = PerceivedObject()
            object.name = c['name']
            object.num_points = int(c['num_points'])
            object.color.r = int(c['color'][0])
            object.color.g = int(c['color'][1])
            object.color.b = int(c['color'][2])
            object.position.x = c['position'][0]
            object.position.y = c['position'][1]
            object.position.z = c['position'][2]
            px, py = self.get_pixel_from3d(c['position'])
            for d in self.detections:
                if px in range(d.xmin, d.xmax) and py in range(d.ymin, d.ymax):
                    object.label = d.Class
                    object.probability = d.probability
                    object.bbox = [d.xmin, d.ymin, d.xmax, d.ymax]
                    print('Cluster {} is a {}'.format(c['name'], d.Class))
            #pos = self.transform_frame(object, self.cam_frame, self.arm_frame)
            ## TODO: Make sure the positions match!!!
            object.position.x = arm_c['position'][0]
            object.position.y = arm_c['position'][1]
            object.position.z = arm_c['position'][2]
            objects.detected_objects.append(object)
        return objects

    def generateSceneGraph(self, objects):
        """
        Scene graphs are generated from the detection and returned in a graph
        
        """
        sceneGraph = Graph()
        
        observation = URIRef("http://example.org/Observation/{}-{}".format(len(objects), time.time()))
        sceneGraph.add((observation, RDF.type, SOSA.Observation)) ##TODO: ADD that it's made by the camera
        for object in objects:
            objNode = URIRef("http://example.org/Objects/{}-{}".format(object.label, time.time()))
            sceneGraph.add((observation, SOSA.hasResult, objNode))
            for prop in list(object.prop.keys()):
                pred = self.visionNS + prop
                sceneGraph.add((objNode, pred, Literal(object.prop[prop])))
        return sceneGraph


    def start(self):
        while not rospy.is_shutdown():
            self.detections = self.yolo.getDetected()
            #self.show_clusters(self.bot, self.cam_frame)
            objects = self.get_objects()
            #if self.image is not None:
            #    self.pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))
            if objects is not None:
                self.detect_pub.publish(objects)
            self.loop_rate.sleep()
            #rospy.loginfo('Running..')

def main(args):   
    try:
        VisionNode()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down perception node.")

if __name__ == '__main__':
    main(sys.argv)        