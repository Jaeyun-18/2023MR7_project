import cv2
from mp_handler import *
import numpy as np

left_landmarks = landmark_translate(True, ["S1", "E1", "W1", "H1"])
center_landmarks = landmark_translate(
    True, ["S1", "E1", "W1", "H1", "S2", "E2", "W2", "H2"])

left = PoseGetter(4, "left", left_landmarks)
center = PoseGetter(
    0, "center", center_landmarks)


while left.is_open():
    left_points, left_img = left.run_cycle(True)
    center_points, center_img = center.run_cycle(True)

    cv2.imshow('center image', cv2.flip(left_img, 1))
    cv2.imshow('left image', cv2.flip(center_img, 1))

    try:
        print(left_points[1], center_points[1])
    except:
        pass
    if cv2.waitKey(5) & 0xFF == 27:
        break
