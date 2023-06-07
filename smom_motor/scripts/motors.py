#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import String
import numpy as np
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from detection_msgs.msg import PerceivedObject, PerceivedObjects

class Motors():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # Initialize node
        rospy.init_node("motors")
        self.bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s", init_node=False)
        self.counter = 0
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(100)
        self.object = None
        # Subsribers
        rospy.Subscriber("workmem_to_motors", PerceivedObject, self.from_wm_cb)
        rospy.Subscriber("/standard_model/perception_output", PerceivedObjects, self.perception_cb)

        self.start()

    def perception_cb(self, msg):
        '''
        Receives the objects from the perception module and generates the scene graph
        '''
        self.objects = msg.detected_objects

    def pickup(self, obj):
        # pick up each object from left-to-right and drop it in a virtual basket on the left side of the robot
        x = obj.position.x
        y = obj.position.y
        z = obj.position.z
        self.bot.arm.set_ee_pose_components(x=x, y=y+0.01, z=z+0.05, pitch=0.5)
        self.bot.arm.set_ee_pose_components(x=x, y=y+0.01, z=z-0.02, pitch=0.5)
        self.bot.gripper.close()
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
        self.bot.arm.set_ee_pose_components(y=0.3, z=0.2)
        self.bot.gripper.open()
        self.bot.arm.set_ee_pose_components(x=0.3, z=0.2)
        self.bot.arm.go_to_sleep_pose()

    def from_wm_cb(self, obj):
        rospy.logerr('Received object: {}'.format(obj.label))
        self.object = obj

    def start(self):
        while not rospy.is_shutdown():
            self.bot.camera.pan_tilt_move(0, 0.75)
            
            if self.object:
                self.pickup(self.object)
                self.object = None
            self.loop_rate.sleep()


def main(args):   
    try:
        Motors()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down motor node.")

if __name__ == '__main__':
    main(sys.argv)                      