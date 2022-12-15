import math
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS
import numpy as np


# This script uses the perception pipeline to pick up objects and place them in some virtual basket on the left side of the robot
# It also uses the AR tag on the arm to get a better idea of where the arm is relative to the camera (though the URDF is pretty accurate already).
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200 use_perception:=true'
# Then change to this directory and type 'python pick_place_no_armtag.py'





def get_pixel_from3d(coords):
    ''''
    Takes as input 3D coordinates, outputs two pixels
    '''
    # Intrinsic matrix - hardcoded for now, TODO: Change it to subscription
    coords = np.append(coords, 1)
    print(coords)
    P = np.array([380.7534484863281, 0.0, 318.1063232421875, 0.0, 0.0, 380.7534484863281, 235.16383361816406, 0.0, 0.0, 0.0, 1.0, 0.0], dtype=np.float32).reshape(3,4)
    print('P', P)
    pix = np.matmul(P, coords)
    return pix.round()[:2]
    #print(pix.shape)
    #print(pix[0])
    #print(pix)
    #uv = [pix[0]/pix[2], pix[1]/pix[2]]
    #print(uv)
    #return uv

def main():
    bot = InterbotixLocobotXS("locobot_wx250s", arm_model="mobile_wx250s")
    bot.camera.pan_tilt_move(0, 0.75)
    #show_clusters(bot, 'locobot/arm_base_link')
    show_clusters(bot, 'locobot/camera_color_optical_frame')
    
    #bot.camera.pan_tilt_move(0.2, 0.60)
    #show_clusters(bot, 'locobot/arm_base_link')
    #show_clusters(bot, 'locobot/camera_color_optical_frame')
    coords = [0.41, 0.03, -0.03]
    get_pixel_from3d(coords)
    #bot.camera.pan_tilt_move(0, 0.75)

if __name__=='__main__':
    main()