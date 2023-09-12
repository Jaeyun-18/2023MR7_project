import cv2
from mp_handler import *

left = PoseGetter(4, "left", (11, 13, 15, 12, 23))
center = PoseGetter(0, "center", (11, 12, 13, 14, 15, 16, 23, 24))

while left.is_open():
    left_points, left_img = left.run_cycle(True)
    center_points, center_img = center.run_cycle(True)

    cv2.imshow('center image', cv2.flip(left_img, 1))
    cv2.imshow('left image', cv2.flip(center_img, 1))

    if cv2.waitKey(5) & 0xFF == 27:
        break
