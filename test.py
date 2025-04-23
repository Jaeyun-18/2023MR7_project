import os
import math
import queue
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

import msvcrt
def getch():
    return msvcrt.getch().decode()

def angle(value):
    return (value+150)/300*1024

def translator(angle_list,act_angle_list):
    DXL_ID= [0,2,4,6,1,3,5,7] # right - even, left - odd
    for i in range(len(DXL_ID)):
        act_angle_list.append([])
        for j in angle_list[i]:
            act_angle_list[-1].append(math.floor(angle(j)))    
    
    return act_angle_list

    # Control table address for Dynamixel MX
TORQUE_ENABLE_AX       = 24               # Control table address is different in Dynamixel model
GOAL_POSITION_AX       = 30
PRESENT_POSITION       = 36

# Protocol version
PROTOCOL_VERSION           = 1.0               #  See which protocol version is used in the Dynamixel

# Default setting 


BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque 
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel MX moving status threshold

index = 0
goal_angle = [ # initial angle reset(차렷 자세)
        [0], #0
        [-90], #2 # right arm 
        [0], #4
        [0], #6
        [0], #1
        [90], #3
        [0], #5 # left arm
        [0]  #7
    ]

initial_angle=[ # initial angle reset(차렷 자세)
        [0], #0
        [-90], #2 # right arm 
        [0], #4
        [0], #6
        [0], #1
        [90], #3
        [0], #5 # left arm
        [0]  #7
    ]

DXL_ID= [0,2,4,6,1,3,5,7] # right - even, left - odd

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

initial_angle_position = []
initial_angle_position = translator(initial_angle, initial_angle_position)

while 1:
    dxl_goal_position = []
    dxl_goal_position = translator(goal_angle, dxl_goal_position)
    print("Press any key to continue! (or press ESC to quit!)")
    inputKey=getch()
    if inputKey == chr(0x1b): # 0x1b = "esc"
        break
    elif inputKey=='r': #press key 'r' angle return to initial position
        dxl_goal_position = initial_angle_position # angle reset as initial position
    elif inputKey=='a':
        goal_angle[0][0] += 10
        dxl_goal_position = []
        dxl_goal_position = translator(goal_angle, dxl_goal_position)
    elif inputKey=='s':
        goal_angle[0][0] -= 10
        dxl_goal_position = []
        dxl_goal_position = translator(goal_angle, dxl_goal_position)
    while 1:
        # Write Dynamixel#1 goal position
        for ID_number in DXL_ID:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID_number, GOAL_POSITION_AX, dxl_goal_position[DXL_ID.index(ID_number)][index])

        memo=[500 for i in range(len(DXL_ID))]
        memo[DXL_ID.index(6)]=2048
        while 1:
            # Read Dynamixel#1 present position
            for ID_number in DXL_ID:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, ID_number, PRESENT_POSITION)
                dxl_torque_enable, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, ID_number, TORQUE_ENABLE)
            result=1

            for ID_number in DXL_ID: 
                result=result*(abs(dxl_goal_position[DXL_ID.index(ID_number)][index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)

            if not result:
                break
        # Change goal position
        if index == len(dxl_goal_position[0])-1:
            index = 0
            break
        else:
            index +=1

# Close port
portHandler.closePort()