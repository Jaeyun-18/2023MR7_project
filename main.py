import cv2
from mp_handler import *

left_landmarks = landmark_translate(True, ["S1", "E1", "W1", "H1"])

left = PoseGetter(4, "left", left_landmarks)
center = PoseGetter(
    0, "center", ["S1", "E1", "W1", "H1", "S2", "E2", "W2", "H2"])


while left.is_open():
    left_points, left_img = left.run_cycle(True)
    center_points, center_img = center.run_cycle(True)

    cv2.imshow('center image', cv2.flip(left_img, 1))
    cv2.imshow('left image', cv2.flip(center_img, 1))

    if cv2.waitKey(5) & 0xFF == 27:
        break