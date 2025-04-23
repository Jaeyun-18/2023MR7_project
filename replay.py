import numpy as np
import time
from dynamixel_sdk import *

TORQUE_ENABLE_AX = 64
GOAL_POSITION_AX = 116
PRESENT_POSITION = 132
CURRENT_LIMIT = 60

PROTOCOL_VERSION = 2.0
BAUDRATE = 3000000
DEVICENAME = '/dev/ttyDYNAMIXEL'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

def replay_motion(file_path, portHandler, packetHandler, DXL_ID, state):
    try:
        motion_data = np.load(file_path)
        print(f"▶️Replaying motion: {file_path} ({motion_data.shape[0]} frames)")
    except Exception as e:
        print(f"❌ Failed to load motion file: {e}")
        return

    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, GOAL_POSITION_AX, 4)

    for frame in motion_data:
        if not state:
            return

        for idx, ID_number in enumerate(DXL_ID):
            try:
                pos = int(frame[idx])
                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(pos)),
                    DXL_HIBYTE(DXL_LOWORD(pos)),
                    DXL_LOBYTE(DXL_HIWORD(pos)),
                    DXL_HIBYTE(DXL_HIWORD(pos))
                ]
                groupSyncWrite.addParam(ID_number, param_goal_position)
            except IndexError:
                print(f"❗ Frame index {idx} out of bounds")
                return

        groupSyncWrite.txPacket()
        groupSyncWrite.clearParam()
        time.sleep(0.05)

    print("✅ Replay finished.")

# Dynamixel 연결
DXL_ID = [0, 2, 4, 6, 1, 3, 5, 7, 8, 9]

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if portHandler.openPort():
    print("✅ Port opened")
else:
    print("❌ Failed to open port")
    exit()

if portHandler.setBaudRate(BAUDRATE):
    print("✅ Baudrate set")
else:
    print("❌ Failed to set baudrate")
    exit()

for ID in DXL_ID:
    packetHandler.write2ByteTxRx(portHandler, ID, CURRENT_LIMIT, 300)
    packetHandler.write1ByteTxRx(portHandler, ID, TORQUE_ENABLE_AX, TORQUE_ENABLE)

# 모션 재생 루프
while True:
    try:
        user_input = input("Enter Motion ID (1 for new_copymotion, 2 for bangbangbang): ").strip()
        if user_input == "1":
            replay_motion("new_copymotion.npy", portHandler, packetHandler, DXL_ID, True)
        elif user_input == "2":
            replay_motion("bangbangbang.npy", portHandler, packetHandler, DXL_ID, True)
        else:
            print("❗ Invalid Motion ID. Please enter 1 or 2.")
    except KeyboardInterrupt:
        print("\nExiting…")
        break

# 포트 닫기
portHandler.closePort()