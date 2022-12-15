import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from interbotix_perception_modules.yolo import InterbotixYoloInterface
from rdflib import Graph, Literal, RDF, URIRef, Namespace
from rdflib.namespace import SOSA

class DetectedObject():
    def __init__(self, pos=None, name=None, color=None, npoints=None, prob=None, bbox=None, label=None):
        self.name = name
        self.position = pos
        self.numberOfPoints = npoints
        self.label=label
        self.prob = prob
        self.bbox = bbox
        self.color = color

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

        # Publishers
        self.pub = rospy.Publisher('test_image', Image,queue_size=20)

        # Subscribers
        rospy.Subscriber("/locobot/camera/color/image_raw",Image,self.callback)

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def show_clusters(self, bot, rf):
        _, self.clusters = bot.pcl.get_cluster_positions(ref_frame=rf, sort_axis="y", reverse=True)
        for cluster in self.clusters:
            print(np.array(cluster['position']).round(2))
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
            objects = []
            for c in self.clusters:
                objects.append(DetectedObject(pos=c['position'],
                name=c['name'], npoints=c['num_points'], color=c['color']))
                px, py = self.get_pixel_from3d(c['position'])
                for d in self.detections:
                    if px in range(d.xmin, d.xmax) and py in range(d.ymin, d.ymax):
                        objects[-1].name = d.Class
                        objects[-1].prob = d.probability
                        objects[-1].bbox = [d.xmin, d.ymin, d.xmax, d.ymax]
                        print('Cluster {} is a {}'.format(c['name'], d.Class))
            return objects

    def generateSceneGraph(self, objects):
        """
        Scene graphs are generated from the detection and returned in a graph
        
        """
        sceneGraph = Graph()

        for cluser in self.clusters:
            observation = URIRef("http://example.org/Observation/" + cluser['name'])
            sceneGraph.add((observation, RDF.type, SOSA.Observation))
            sceneGraph.add((observation, self.locobot.hasPosition, Literal(str(cluser['position']))))
            sceneGraph.add((observation, self.locobot.hasColor, Literal(str(cluser['color']))))
            sceneGraph.add((observation, self.locobot.hasNumberOfPoints, Literal(str(cluser['num_points']))))
        for detection in self.detections:
            bbox = [detection.xmin, detection.ymin, detection.xmax, detection.ymax]
            observation = URIRef("http://example.org/Observation/" + detection.Class)
            sceneGraph.add((observation, RDF.type, SOSA.Observation))
            sceneGraph.add((observation, self.locobot.hasLabel, Literal(detection.Class)))
            sceneGraph.add((observation, self.locobot.hasProbability, Literal(detection.probability)))
            sceneGraph.add((observation, self.locobot.hasBoundingBox, Literal(bbox)))
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
            print(self.detections)
            self.show_clusters(self.bot, 'locobot/camera_color_optical_frame')
            self.objects = self.get_objects()
            if z >= z_uplimit or z <= z_lowlimit: d*=-1
            z+= d
            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))
            self.bot.camera.pan_tilt_move(z, i)
            self.loop_rate.sleep()

if __name__ == '__main__':
    #rospy.init_node("imagetimer111", anonymous=True)
    my_node = VisionNode()
    my_node.start()