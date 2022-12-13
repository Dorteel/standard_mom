import math
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script uses the perception pipeline to pick up objects and place them in some virtual basket on the left side of the robot
# It also uses the AR tag on the arm to get a better idea of where the arm is relative to the camera (though the URDF is pretty accurate already).
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200 use_perception:=true'
# Then change to this directory and type 'python pick_place_no_armtag.py'


def show_clusters(bot, rf):
    _, clusters = bot.pcl.get_cluster_positions(ref_frame=rf, sort_axis="y", reverse=True)
    print('{}\n{}\n{}'.format('='*len(rf), rf, '='*len(rf)))
    for cluster in clusters:
        print('\t{}\n'.format([round(x,2) for x in cluster["position"]]))

def main():
    bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")
    bot.camera.pan_tilt_move(0, 0.75)
    show_clusters(bot, 'locobot/arm_base_link')
    show_clusters(bot, 'locobot/camera_color_optical_frame')
    
    bot.camera.pan_tilt_move(0.2, 0.60)
    show_clusters(bot, 'locobot/arm_base_link')
    show_clusters(bot, 'locobot/camera_color_optical_frame')

    bot.camera.pan_tilt_move(0, 0.75)

if __name__=='__main__':
    main()