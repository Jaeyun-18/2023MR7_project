import cv2 as cv
import glob
import numpy as np
from typing import Tuple

ROWS = 7
COLUMNS = 9


class StereoCameraSystem:
    def __init__(self, img1_name: str, img2_name: str, img1_dir: str, img2_dir: str, sync_dir: str, checkerborad_size: Tuple[int, int]):
        self.COLUMNS, self.ROWS = checkerborad_size
        self.img1_name = img1_name
        self.img2_name = img2_name
        self.img1_dir = img1_dir
        self.img2_dir = img2_dir
        self.sync_dir = sync_dir

    def calibrate(self, load: bool, dir: str):
        if load == True:
            l = np.load(dir)
            self.R = l['R']
            self.T = l['T']
            self.mtx1 = l[self.img1_name]
            self.mtx2 = l[self.img2_name]
            return

        self.mtx1, dist1 = self.__calibrate_single_camera(self.img1_dir)
        self.mtx2, dist2 = self.__calibrate_single_camera(self.img2_dir)
        cv.destroyAllWindows()

        self.R, self.T = self.__stereo_calibrate(
            self.mtx1, dist1, self.mtx2, dist2, self.sync_dir)

        vals_to_save = {'R': self.R, 'T': self.T,
                        self.img1_name: self.mtx1, self.img2_name: self.mtx2}

        np.savez(dir, **vals_to_save)

        cv.destroyAllWindows()

    def __calibrate_single_camera(self, images_folder):
        images_names = glob.glob(images_folder + "/*")
        images = []
        for imname in images_names:
            im = cv.imread(imname, 1)
            images.append(im)

        # criteria used by checkerboard pattern detector.
        # Change this if the code can't find the checkerboard
        criteria = (cv.TERM_CRITERIA_EPS +
                    cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        world_scaling = 1.  # change this to the real world square size. Or not.

        # coordinates of squares in the checkerboard world space
        objp = np.zeros((self.ROWS*self.COLUMNS, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.ROWS, 0:self.COLUMNS].T.reshape(-1, 2)
        objp = world_scaling * objp

        # frame dimensions. Frames should be the same size.
        width = images[0].shape[1]
        height = images[0].shape[0]

        # Pixel coordinates of checkerboards
        imgpoints = []  # 2d points in image plane.

        # coordinates of the checkerboard in checkerboard world space.
        objpoints = []  # 3d point in real world space

        for frame in images:
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # find the checkerboard
            ret, corners = cv.findChessboardCorners(
                gray, (self.ROWS, self.COLUMNS), None)
            if ret == True:

                # Convolution size used to improve corner detection. Don't make this too large.
                conv_size = (11, 11)

                # opencv can attempt to improve the checkerboard coordinates
                corners = cv.cornerSubPix(
                    gray, corners, conv_size, (-1, -1), criteria)
                cv.drawChessboardCorners(
                    frame, (self.ROWS, self.COLUMNS), corners, ret)
                cv.imshow('img', frame)
                cv.waitKey(100)

                objpoints.append(objp)
                imgpoints.append(corners)

        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            objpoints, imgpoints, (width, height), None, None)
        print('rmse:', ret)
        print('camera matrix:\n', mtx)
        print('distortion coeffs:', dist)
        print('Rs:\n', rvecs)
        print('Ts:\n', tvecs)

        return mtx, dist

    def __stereo_calibrate(self, mtx1, dist1, mtx2, dist2, frames_folder):
        # read the synched frames
        c1_images_names = sorted(
            glob.glob(frames_folder + "/" + self.img1_name + "*"))
        c2_images_names = sorted(
            glob.glob(frames_folder + "/" + self.img2_name + "*"))

        print(c1_images_names)
        print(c2_images_names)

        c1_images = []
        c2_images = []
        for im1, im2 in zip(c1_images_names, c2_images_names):
            _im = cv.imread(im1, 1)
            c1_images.append(_im)

            _im = cv.imread(im2, 1)
            c2_images.append(_im)

        # change this if stereo calibration not good.
        criteria = (cv.TERM_CRITERIA_EPS +
                    cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)

        world_scaling = 1.  # change this to the real world square size. Or not.

        # coordinates of squares in the checkerboard world space
        objp = np.zeros((self.ROWS*self.COLUMNS, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.ROWS, 0:self.COLUMNS].T.reshape(-1, 2)
        objp = world_scaling * objp

        # frame dimensions. Frames should be the same size.
        width = c1_images[0].shape[1]
        height = c1_images[0].shape[0]

        # Pixel coordinates of checkerboards
        imgpoints_left = []  # 2d points in image plane.
        imgpoints_right = []

        # coordinates of the checkerboard in checkerboard world space.
        objpoints = []  # 3d point in real world space

        print("============", len(c1_images))
        i = 0
        
        for frame1, frame2 in zip(c1_images, c2_images):
            size = (self.ROWS, self.COLUMNS)
            gray1 = cv.cvtColor(frame1, cv.COLOR_BGR2GRAY)
            gray2 = cv.cvtColor(frame2, cv.COLOR_BGR2GRAY)
            c_ret1, corners1 = cv.findChessboardCornersSB(gray1, size, None)
            c_ret2, corners2 = cv.findChessboardCornersSB(gray2, size, None)
            print("============", i, c_ret1, c_ret2)
            i += 1

            if c_ret1 == True and c_ret2 == True:
                corners1 = cv.cornerSubPix(
                    gray1, corners1, (11, 11), (-1, -1), criteria)
                corners2 = cv.cornerSubPix(
                    gray2, corners2, (11, 11), (-1, -1), criteria)

                cv.drawChessboardCorners(frame1, size, corners1, c_ret1)
                cv.imshow(self.img1_name, frame1)

                cv.drawChessboardCorners(frame2, size, corners2, c_ret2)
                cv.imshow(self.img2_name, frame2)

                if cv.waitKey(5000) == ord('f'):
                    corners1 = np.flip(corners1, axis=0)
                    cv.drawChessboardCorners(frame1, size, corners1, c_ret1)
                    cv.imshow(self.img1_name, frame1)

                    cv.drawChessboardCorners(frame2, size, corners2, c_ret2)
                    cv.imshow(self.img2_name, frame2)

                objpoints.append(objp)
                imgpoints_left.append(corners1)
                imgpoints_right.append(corners2)

        stereocalibration_flags = cv.CALIB_FIX_INTRINSIC
        ret, CM1, dist1, CM2, dist2, R, T, E, F = cv.stereoCalibrate(objpoints, imgpoints_left, imgpoints_right, mtx1, dist1,
                                                                     mtx2, dist2, (width, height), criteria=criteria, flags=stereocalibration_flags)

        print(ret)
        return R, T

    def triangulate(self, img1_points, img2_points):
        # RT matrix for C1 is identity.
        RT1 = np.concatenate([np.eye(3), [[0], [0], [0]]], axis=-1)
        P1 = self.mtx1 @ RT1  # projection matrix for C1

        # RT matrix for C2 is the R and T obtained from stereo calibration.
        RT2 = np.concatenate([self.R, self.T], axis=-1)
        P2 = self.mtx2 @ RT2  # projection matrix for C2

        def DLT(P1, P2, point1, point2):

            A = [point1[1]*P1[2, :] - P1[1, :],
                 P1[0, :] - point1[0]*P1[2, :],
                 point2[1]*P2[2, :] - P2[1, :],
                 P2[0, :] - point2[0]*P2[2, :]
                 ]
            A = np.array(A).reshape((4, 4))
            # print('A: ')
            # print(A)

            B = A.transpose() @ A
            from scipy import linalg
            U, s, Vh = linalg.svd(B, full_matrices=False)
            return Vh[3, 0:3]/Vh[3, 3]

        p3ds = []
        for right_point, left_point in zip(img1_points, img2_points):
            _p3d = DLT(P1, P2, right_point, left_point)
            p3ds.append(_p3d)
        p3ds = np.array(p3ds)

        return p3ds


if __name__ == "__main__":
    scs = StereoCameraSystem("right_camera", "left_camera", "cali_imgs/right_imgs",
                             "cali_imgs/left_imgs", "cali_imgs/sync_imgs", [7, 9])
    scs.calibrate(False, "test_mtx.npz")
