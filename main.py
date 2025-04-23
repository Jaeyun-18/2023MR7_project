import cv2
from mp_handler import *
import numpy as np
from typing import Tuple
import threading
import queue
import os

os.sys.path.append('../DynamixelSDK/python/build')
from dynamixel_sdk import *

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

# q = queue.Queue()
# thread_motor = threading.Thread(target=mov_motor, args=(q,))
# thread_motor.daemon = True
# thread_motor.start()

# Control table address is different in Dynamixel model
TORQUE_ENABLE_AX = 24
GOAL_POSITION_AX = 30
PRESENT_POSITION = 36
TORQUE_LIMIT = 34

# Protocol version
PROTOCOL_VERSION = 1.0  # See which protocol version is used in the Dynamixel

# Default setting

BAUDRATE = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB0'    # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE = 1                 # Value for enabling the torque
TORQUE_DISABLE = 0                 # Value for disabling the torque
# Dynamixel MX moving status threshold
DXL_MOVING_STATUS_THRESHOLD = 100

goal_angle = np.array([  # initial angle reset(차렷 자세)
    [0],  # 0
    [-90],  # 2 # right arm
    [0],  # 4
    [0],  # 6
    [0],  # 1
    [90],  # 3
    [0],  # 5 # left arm
    [0]  # 7
])

initial_angle = np.array([  # initial angle reset(차렷 자세)
    [0],  # 0
    [-90],  # 2 # right arm
    [0],  # 4
    [0],  # 6
    [0],  # 1
    [90],  # 3
    [0],  # 5 # left arm
    [0]  # 7
])

DXL_ID = [0, 2, 4, 6, 1, 3, 5, 7]  # right - even, left - odd
index = 0

# Initialize PortHandler instance
# Set the port paths
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

if portHandler.openPort():
    pass
# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    pass


initial_angle_position = initial_angle.astype(int)
for ID_number in DXL_ID:
    r, e = packetHandler.write2ByteTxRx(
        portHandler, ID_number, TORQUE_LIMIT, 256)

while left.is_open() and right.is_open():
    try:
        left_points, left_img = left.run_cycle()
        right_points, right_img = right.run_cycle()
        goal_angle = cal_angle(
            TestCamSys.triangulate(right_points, left_points))

        # if np.any(np.abs(goal_angle - pre_goal_angle) > 20):
        #     # q.put(goal_angle)
        #     pre_goal_angle = goal_angle

        dxl_goal_position = goal_angle.astype(int)
        for ID_number in DXL_ID:
            dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
                portHandler, ID_number, GOAL_POSITION_AX, dxl_goal_position[DXL_ID.index(ID_number)][index])

            # Change goal position

        right.show_vid(None)
        left.show_vid(None)

    except Exception as e:
        print(e)

    if cv2.waitKey(5) == ord('q'):
        break

portHandler.closePort()
