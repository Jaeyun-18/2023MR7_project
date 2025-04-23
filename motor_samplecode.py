import os
import math
import queue
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# Control table address for Dynamixel MX
TORQUE_ENABLE_AX       = 64              # Control table address is different in Dynamixel model
GOAL_POSITION_AX       = 116
PRESENT_POSITION       = 132

# Protocol version
PROTOCOL_VERSION           = 2.0               #  See which protocol version is used in the Dynamixel

# Default setting 


BAUDRATE                    = 3000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque 
DXL_MOVING_STATUS_THRESHOLD = 10               # Dynamixel MX moving status threshold


index = 0
def angle(value):
    # return (value+150)/300*1024
    return (value / 360) * 4096 + 2048

def translator(angle_list,act_angle_list):
    for i in range(len(DXL_ID)):
        act_angle_list.append([])
        for j in angle_list[i]:
            act_angle_list[-1].append(int(angle(j)))  # floor 대신 int도 충분
    
    return act_angle_list

DXL_ID= [0,2,4,6,1,3,5,7,8,9] # right - even, left - odd
goal_angle = [ # initial angle reset(차렷 자세)
        [0], #0
        [-90], #2 # right arm 
        [0], #4
        [0], #6
        [0], #1
        [90], #3 # left arm
        [0], #5
        [0],  #7
        [90], #8 # right arm
        [-90] #9 # left arm
    ]

initial_angle=[ # initial angle reset(차렷 자세)
        [0], #0
        [-90], #2 # right arm 
        [0], #4
        [0], #6
        [0], #1
        [90], #3 # left arm
        [0], #5
        [0],  #7
        [90], #8 # right arm
        [-90] #9 # left arm
    ]

# Initialize PortHandler instance
# Set the port paths
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel#1 Torque
for ID_number in DXL_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_number, TORQUE_ENABLE_AX, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % ID_number)

# interval=25
# scale=3

while 1:
    initial_angle_position = []
    initial_angle_position = translator(initial_angle, initial_angle_position)

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
    for ID_number in range(4):
        if(dxl_goal_position[ID_number][0]>4095):
            dxl_goal_position[ID_number][0]=4095
        elif(dxl_goal_position[ID_number][0]<0):
            dxl_goal_position[ID_number][0]=0

    while 1:
        # print(goal_angle)
        # Write Dynamixel#1 goal position
        for ID_number in DXL_ID:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID_number, GOAL_POSITION_AX, dxl_goal_position[DXL_ID.index(ID_number)][index])
            print("dxl_goal_position" + str(ID_number) + ":" + str(dxl_goal_position[DXL_ID.index(ID_number)][index]))
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0: 
                print("%s" % packetHandler.getRxPacketError(dxl_error))

        while 1:
            # Read Dynamixel#1 present position
            for ID_number in DXL_ID:
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, ID_number, PRESENT_POSITION)
                dxl_torque_enable, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, ID_number, TORQUE_ENABLE)

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))
                
                
                print("[ID:%03d] GoalPos:%.02f  PresPos:%.02f" % (ID_number, (dxl_goal_position[DXL_ID.index(ID_number)][index]), dxl_present_position),end='|  ')
                print("torque: %d" % (dxl_torque_enable,))
            print(' ')
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


# Disable Dynamixel#1 Torque
for ID_number in DXL_ID:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID_number, TORQUE_ENABLE_AX, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))


# Close port
portHandler.closePort() 