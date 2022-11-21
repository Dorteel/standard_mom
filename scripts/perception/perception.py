#!/usr/bin/env python3

import roslaunch
import rospy
import os

rospy.init_node('Perception', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
pwd = os.path.dirname(os.path.abspath(__file__))
launch = roslaunch.parent.ROSLaunchParent(uuid, ['/'.join(str(pwd).split('/')[:-2]) + '/launch/perception.launch'])
launch.start()


try:
  launch.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  launch.shutdown()
# 3 seconds later
# launch.shutdown()