import cv2
from mp_handler import *
import numpy as np

left_landmarks = landmark_translate(True, ["S1", "S2", "E1", "W1"])
front_landmarks = landmark_translate(
    True, ["S1", "S2", "E1", "E2", "W1", "W2", "H1", "H2"])

left = PoseGetter(3, "left", left_landmarks)
front = PoseGetter(0, "front", front_landmarks)


while left.is_open():
    left_points, left_img = left.run_cycle(True)
    front_points, front_img = front.run_cycle(True)

    S1_F = front_points[0]
    S2_F = front_points[1]
    E1_F = front_points[2]
    E2_F = front_points[3]
    W1_F = front_points[4]
    W2_F = front_points[5]
    H1_F = front_points[6]
    H2_F = front_points[7]

    S1_L = left_points[0]
    S2_L = left_points[1]
    E1_L = left_points[2]
    W1_L = left_points[3]

    cv2.imshow('left image', cv2.flip(left_img, 1))
    cv2.imshow('front image', cv2.flip(front_img, 1))

    try:
        print(left_points[1], front_points[1])
    except:
        pass

    if cv2.waitKey(5) & 0xFF == 27:
        break
