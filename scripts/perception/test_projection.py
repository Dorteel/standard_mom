import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from interbotix_xs_modules.locobot import InterbotixLocobotXS

class Node(object):
    def __init__(self):
        # Params
        self.bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")

        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)

        # Publishers
        self.pub = rospy.Publisher('test_image', Image,queue_size=20)

        # Subscribers
        rospy.Subscriber("/locobot/camera/color/image_raw",Image,self.callback)

    def callback(self, msg):
        self.image = self.br.imgmsg_to_cv2(msg)

    def show_clusters(self, bot, rf):
        _, clusters = bot.pcl.get_cluster_positions(ref_frame=rf, sort_axis="y", reverse=True)
        for cluster in clusters:
            print(np.array(cluster['position']).round(2))
            pixels = self.get_pixel_from3d(cluster['position'])
            # Draw on image the pixel coordinates
            self.image = cv2.circle(self.image, pixels, 10, (255, 0, 0), 3)

            
    def get_pixel_from3d(self, coords):
        ''''
        Takes as input 3D coordinates, outputs two pixels
        '''
        # Intrinsic matrix - hardcoded for now, TODO: Change it to subscription
        coords = np.append(coords, 1)
        P = np.array([380.7534484863281, 0.0, 318.1063232421875, 0.0, 0.0, 380.7534484863281, 235.16383361816406, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32).reshape(3,4)
        pix = np.matmul(P, coords)
        c = np.array([pix[0]/pix[2], pix[1]/pix[2]]).astype('int32')
        return c


    def start(self):
        i_uplimit = 1
        i = 0.8
        d = 0.05
        i_lowlimit = 0.6
        z = 0
        z_uplimit = 0.25
        z_lowlimit = -0.25
        while not rospy.is_shutdown():
            self.show_clusters(self.bot, 'locobot/camera_color_optical_frame')
            if z >= z_uplimit or z <= z_lowlimit: d*=-1
            z+= d
            if self.image is not None:
                self.pub.publish(self.br.cv2_to_imgmsg(self.image, "bgr8"))
            self.bot.camera.pan_tilt_move(z, i)
            self.loop_rate.sleep()

if __name__ == '__main__':
    #rospy.init_node("imagetimer111", anonymous=True)
    my_node = Node()
    my_node.start()