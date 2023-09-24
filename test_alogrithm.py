import cv2
from mp_handler import *
import numpy as np

left_landmarks = landmark_translate(True, ["H1", "S1", "E1", "W1"])
front_landmarks = landmark_translate(
    True, ["H1", "S1", "E1", "W1", "H2", "S2", "E2", "W2"])

left = PoseGetter(4, "left", left_landmarks)
front = PoseGetter(
    0, "center", front_landmarks)


while left.is_open():
    left_points, left_img = left.run_cycle(True)
    front_points, front_img = front.run_cycle(True)

    cv2.imshow('center image', cv2.flip(left_img, 1))
    cv2.imshow('left image', cv2.flip(front_img, 1))

    try:
        print(left_points[1], front_points[1])
    except:
        pass
    if cv2.waitKey(5) & 0xFF == 27:
        break
