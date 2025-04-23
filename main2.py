import cv2
from mp_handler import *
import numpy as np
from typing import Tuple
import multiprocessing
import queue

from stereo_camera import StereoCameraSystem
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from new_functions import *
from motor_control import *

landmarks = landmark_translate(
    True, ["W1", "E1", "S1", "H1", "H2", "S2", "E2", "W2"])
          # 0     1     2     3     4     5     6     7

right = PoseGetter(4, "right", landmarks, [640, 480])
left = PoseGetter(2, "left", landmarks, [640, 480])

font_size = 0.8

TestCamSys = StereoCameraSystem("right_camera", "left_camera", "cali_imgs/right_imgs",
                                "cali_imgs/left_imgs", "cali_imgs/sync_imgs", [7, 9])
TestCamSys.calibrate(True, "test_mtx.npz")

if __name__ == '__main__':
    pre_goal_angle = np.array([ # initial angle reset(차렷 자세)
            [0], #0
            [-90], #2 # right arm 
            [0], #4
            [0], #6
            [0], #1
            [90], #3
            [0], #5 # left arm
            [0]  #7
        ])
    
    q = queue.Queue()
    process_motor = multiprocessing.Process(target=mov_motor, args=(q,))
    process_motor.daemon = True
    process_motor.start()
    while left.is_open() and right.is_open():
        try:
            left_points, left_img = left.run_cycle()
            right_points, right_img = right.run_cycle()
            goal_angle = cal_angle(TestCamSys.triangulate(right_points, left_points))
            
            if  np.any(np.abs(goal_angle - pre_goal_angle) > 20):
                print(goal_angle)
                q.put(goal_angle)
                pre_goal_angle = goal_angle
            
            right.show_vid(None)
            left.show_vid(None)

        except Exception as e:
            print(e)

        if cv2.waitKey(5) == ord('q'):
            break
