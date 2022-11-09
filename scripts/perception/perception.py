#!/usr/bin/env python3

import roslaunch
import rospy
import os

rospy.init_node('Perception-3', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
pwd = os.getcwd()
rospy.logfatal(str(pwd))
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/haier/catkin_ws/src/testapi/launch/test_node.launch"])
launch.start()
rospy.logfatal("started")

rospy.sleep(3)
# 3 seconds later
launch.shutdown()