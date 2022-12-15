import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from interbotix_perception_modules.yolo import InterbotixYoloInterface
from rdflib import Graph, Literal, RDF, URIRef, Namespace
from rdflib.namespace import SOSA
from detection_msgs.msg import PerceivedObject, PerceivedObjects
import time

class DetectedObject():
    def __init__(self, pos=None, name=None, color=None, npoints=None, prob=None, bbox=None, label=None):
        self.name = name
        self.position = pos
        self.numberOfPoints = npoints
        self.label=label
        self.prob = prob
        self.bbox = bbox
        self.color = color
        self.detectedAt = time.time()
        self.prop = {'hasName' : name, 'hasPosition' : pos, 'hasNumberOfPoints' : npoints, 'hasLabel' : label, 'hasProb' : prob, 'hasBbox' : bbox, 'hasColor' : color}
    
    def sameAs(self, object):
        '''
        Checks whether the object is the same within some boundary
        '''
        pass




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
        self.visionNS = URIRef("http://example.org/VisionProperies/")
        # Initialize prediction publisher
        self.detect_pub = rospy.Publisher("/standard_model/perception_output", PerceivedObjects, queue_size=10)
        self.pub = rospy.Publisher('test_image', Image,queue_size=20)
        
        # Subscribers
        rospy.Subscriber("/locobot/camera/color/image_raw",Image,self.callback)

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def show_clusters(self, bot, rf):
        _, self.clusters = bot.pcl.get_cluster_positions(ref_frame=rf, sort_axis="y", reverse=True)
        for cluster in self.clusters:
            print(cluster)
            #print(np.array(cluster['position']).round(2))
            pixels = self.get_pixel_from3d(cluster['position'])
            # Draw on image the pixel coordinates
            self.image = cv2.circle(self.image, pixels, 10, (255, 0, 0), 3)

            
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
            objects = PerceivedObjects()
            for c in self.clusters:
                object = PerceivedObject()
                object.name = c['name']
                object.num_points = c['num_points']
                object.color = c['color']
                object.position = c['position']
                
                px, py = self.get_pixel_from3d(c['position'])
                for d in self.detections:
                    if px in range(d.xmin, d.xmax) and py in range(d.ymin, d.ymax):
                        object.label = d.Class
                        object.probability = d.probability
                        object.bbox = [d.xmin, d.ymin, d.xmax, d.ymax]
                        print('Cluster {} is a {}'.format(c['name'], d.Class))
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
        i_uplimit = 1
        i = 0.8
        d = 0.05
        i_lowlimit = 0.6
        z = 0
        z_uplimit = 0.25
        z_lowlimit = -0.25
        while not rospy.is_shutdown():
            self.detections = self.yolo.getDetected()
            self.show_clusters(self.bot, 'locobot/camera_color_optical_frame')
            objects = self.get_objects()
            #self.sceneGraph = self.generateSceneGraph(objects)
            #print(self.sceneGraph.serialize(format='ttl'))
            if z >= z_uplimit or z <= z_lowlimit: d*=-1
            z+= d
            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))
            self.detect_pub.publish(objects)
            self.bot.camera.pan_tilt_move(z, i)

            self.loop_rate.sleep()

if __name__ == '__main__':
    #rospy.init_node("imagetimer111", anonymous=True)
    my_node = VisionNode()
    my_node.start()