from __future__ import annotations
import cv2
import numpy as np
import time


class Camera:
    # Todo: undistortPoint -> Projection matrix -> cross = 0
    def __init__(self, pre_set_proj: np.ndarray = None, pre_set_dist: np.ndarray = None, cali_images=None):
        if (pre_set_proj == None) and (pre_set_dist == None) and (cali_images == None):
            raise (
                "No pre setted values and calibration images. Must include at least one of them")
        else:
            if (pre_set_proj != None) and (pre_set_dist != None):
                self.proj = pre_set_proj
                self.dist = pre_set_dist
            else:
                self.calibrate(cali_images)

    def calibrate(self, images):
        CHECKERBOARD = (6, 9)
        criteria = (cv2.TERM_CRITERIA_EPS +
                    cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        objpoints = []

        imgpoints = []

        objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0],
                                  0:CHECKERBOARD[1]].T.reshape(-1, 2)

        for img in images:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray,
                                                     CHECKERBOARD,
                                                     cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                img = cv2.drawChessboardCorners(
                    img, CHECKERBOARD, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(0)
        cv2.destroyAllWindows()
        h, w = img.shape[:2]  # 480, 640

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None)

        print(mtx, dist, rvecs, tvecs)


if __name__ == "__main__":
    vid_obj = cv2.VideoCapture(0)
    frames = []
    t0 = time.time()
    time_delta = 5
    frame_num = 1

    while vid_obj.isOpened():
        ret, frame = vid_obj.read()
        if len(frames) == frame_num:
            print("captured all")
            break

        if ret:
            cv2.imshow("calibration", frame)
            if time.time() - t0 >= time_delta:
                frames.append(frame)
                print("captured {} frame".format(len(frames)))
                t0 = time.time()

            else:
                print("capture in", round(t0 + time_delta - time.time(), 1))

        if cv2.waitKey(5) & 0xFF == 27:
            break

    vid_obj.release()
    cv2.destroyAllWindows()

    c = Camera(cali_images=frames)
