#!/usr/bin/env python3

# from chardet import detect
import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_perception_modules.yolo import InterbotixYoloInterface

class Perception():
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


        # Buffers
        # self.motor_buffer = np.empty()
        # self.perc_buffer = np.empty()
        # self.decl_buffer = np.empty()
        # self.proc_buffer = np.empty()

        self.start()

    def from_wm_cb(self, msg):
        self.msg = msg
        #rospy.loginfo("{}: Received from working memory: {}".format("Perception", self.msg.data))


    def start(self):
        while not rospy.is_shutdown():
            # Publish our custom message.
            #_, self.clusters = self.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
            _, self.clusters = self.pcl.get_cluster_positions(ref_frame="locobot/arm_base_link", sort_axis="y", reverse=True)
            self.detections = self.yolo.getDetected()
            msg =  self.constructPerception(self.clusters, self.detections)

            self.pub.publish(msg)
            self.loop_rate.sleep()


    def constructPerception(self, clusters, detections):
        rospy.logwarn("Clusters: {}".format(clusters))
        rospy.logwarn("Detection: {}".format(detections))
        rospy.logerr("----------")
        return "True"

def main(args):   
    try:
        Perception()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down perception node.")

if __name__ == '__main__':
    main(sys.argv)                      