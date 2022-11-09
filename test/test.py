#!/usr/bin/env python3

import roslaunch
import rospy
import os

rospy.init_node('Perception-3', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
pwd = os.path.dirname(os.path.abspath(__file__))
rospy.logwarn(str(pwd))
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/user/locobot_ws/src/standard_model_of_mind/standard_mom/launch/perception.launch"])
launch.start()
rospy.logwarn("started")

rospy.sleep(3)
# 3 seconds later
# launch.shutdown()