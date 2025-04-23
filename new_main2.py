import cv2
from mp_handler import *
import numpy as np
from typing import Tuple
import threading
import queue
from dynamixel_sdk import *
import ctypes

from stereo_camera import StereoCameraSystem
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from new_functions2 import *
from motor_control2 import *

is_recording = False
recorded_angles = []

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
TORQUE_ENABLE_AX       = 64              # Control table address is different in Dynamixel model
GOAL_POSITION_AX       = 116
PRESENT_POSITION       = 132
CURRENT_LIMIT = 60

# Protocol version
PROTOCOL_VERSION = 2.0  # See which protocol version is used in the Dynamixel

# Default setting

BAUDRATE = 3000000             # Dynamixel default baudrate : 57600
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
    [90],  # 3 # left arm
    [0],  # 5
    [0],  # 7
    [90], # 8 # right arm 2
    [-90]  # 9 # left arm 2
])

initial_angle = np.array([  # initial angle reset(차렷 자세)
    [0],  # 0
    [-90],  # 2 # right arm
    [0],  # 4
    [0],  # 6
    [0],  # 1
    [90],  # 3 # left arm
    [0],  # 5
    [0],  # 7
    [90], # 8 # right arm 2
    [-90]  # 9 # left arm 2
])

DXL_ID = [0, 2, 4, 6, 1, 3, 5, 7, 8, 9]  # right - even, left - odd
index = 0

# Initialize PortHandler instance
# Set the port paths
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, GOAL_POSITION_AX, 4)


if portHandler.openPort():
    pass
# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    pass

initial_angle_position = initial_angle.astype(int)
for ID_number in DXL_ID:
    r, e = packetHandler.write2ByteTxRx(
        portHandler, ID_number, CURRENT_LIMIT, 300)
    packetHandler.write1ByteTxRx(portHandler, ID_number, TORQUE_ENABLE_AX, TORQUE_ENABLE)
# packetHandler.write1ByteTxRx(portHandler, 8, TORQUE_ENABLE_AX, TORQUE_DISABLE)
# packetHandler.write1ByteTxRx(portHandler, 9, TORQUE_ENABLE_AX, TORQUE_DISABLE)

while left.is_open() and right.is_open():
    try:
        left_points, left_img = left.run_cycle()
        right_points, right_img = right.run_cycle()
        goal_angle = cal_angle(
            TestCamSys.triangulate(right_points, left_points))
        if goal_angle.shape[0] < 10:
            # 기존 관절 복사해서 확장
            goal_angle = np.vstack([
                goal_angle,
                [[4096-1*goal_angle[1][0]]],
                [[4096-1*goal_angle[5][0]]]
            ])
        else:
            goal_angle[8][0] = 4096-1*goal_angle[1][0]
            goal_angle[9][0] = 4096-1*goal_angle[5][0]
        # if np.any(np.abs(goal_angle - pre_goal_angle) > 20):
        #     # q.put(goal_angle)
        #     pre_goal_angle = goal_angle

        dxl_goal_position = goal_angle.astype(int)
        if is_recording:
            recorded_angles.append(dxl_goal_position.copy())

        for ID_number in DXL_ID:
            num = dxl_goal_position[DXL_ID.index(ID_number)][index]
            
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(num)),
                DXL_HIBYTE(DXL_LOWORD(num)),
                DXL_LOBYTE(DXL_HIWORD(num)),
                DXL_HIBYTE(DXL_HIWORD(num))
            ]
            dxl_addparam_result = groupSyncWrite.addParam(ID_number, param_goal_position)
            if not dxl_addparam_result:
                print(f"Failed to add param for ID {ID_number}")
            # Change goal position
        groupSyncWrite.txPacket()
        groupSyncWrite.clearParam()

        right.show_vid(None)
        left.show_vid(None)

    except Exception as e:
        print(e)

    key = cv2.waitKey(5) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):
        is_recording = True
        recorded_angles = []  # 초기화
        print("📸 Recording started.")
    elif key == ord('s'):
        is_recording = False
        print(f"💾 Recording stopped. {len(recorded_angles)} frames saved.")
        np.save("recorded_angles.npy", np.array(recorded_angles))

portHandler.closePort()