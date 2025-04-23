import cv2
import numpy as np
from threading import Thread


class Cameras:
    def __init__(self, cam_info, directory):
        self.caps = {}
        self.cam_info = cam_info
        for id, name in cam_info.items():
            self.caps[name] = cv2.VideoCapture(id)
        print(self.caps)
        self.directory = directory

    def run(self):
        c = Thread(target=self.capture)
        c.start()
        self.images = {}
        while True:
            try:
                for id, name in self.cam_info.items():
                    success, image = self.caps[name].read()
                    if not success:
                        # print("camera {} does not work".format(id))
                        pass

                    image.flags.writeable = False
                    self.images[name] = (cv2.flip(image, 1))

                    cv2.imshow(name, self.images[name])
                    cv2.waitKey(1)
            except Exception as e:
                # print(e)
                pass

    def capture(self):
        capture_num = 0
        while True:
            i = input("capture: ")
            if i == 'c':
                for id, name in self.cam_info.items():
                    cv2.imwrite(self.directory +
                                "{}_{}.jpg".format(name, capture_num), self.images[name])
                    print("captured {}_{}.jpg".format(name, capture_num))
                capture_num += 1
            elif i == 'd':
                cv2.destroyAllWindows()
                return


if __name__ == "__main__":

    # left_cam = Cameras({2: 'left_camera'}, "cali_imgs/left_imgs/")
    # left_cam.run()
    
    # right_cam = Cameras({4: 'right_camera'}, "cali_imgs/right_imgs/")
    # right_cam.run()

    both_cam = Cameras({4: 'right_camera', 2: 'left_camera'},"cali_imgs/sync_imgs/")
    both_cam.run() 