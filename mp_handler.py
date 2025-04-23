import cv2
import mediapipe as mp
import numpy as np
from typing import Tuple, Dict


class PoseGetter:
    def __init__(self, camera_num: int, name: str, wanted_points: list, img_size: Tuple[int, int]) -> None:
        self.cap = cv2.VideoCapture(camera_num)
        self.name = name
        self.wanted_points = wanted_points
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.8, min_tracking_confidence=0.8)
        self.img_size = np.array(img_size)

    def is_open(self) -> bool:
        return self.cap.isOpened()

    def run_cycle(self) -> Tuple[np.ndarray, cv2.Mat]:
        success, self.image = self.cap.read()
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        if not success:
            print("camera {} does not work".format(self.name))
            return None
        try:
            self.image.flags.writeable = False
            self.result = self.pose.process(self.image)
            landmarks = self.result.pose_landmarks

            ret = []
            if landmarks != None:
                for k in self.wanted_points:
                    lm = landmarks.landmark[k]
                    ret.append([lm.x, lm.y])
            ret = np.array(ret)
            ret = self.img_size * ret

            return ret, self.image
        except Exception as e:
            print(e)

    def show_vid(self, write_angles: Dict[str, float], font_size: float = 1.0, font_color: Tuple[int, int, int] = (0, 0, 0)) -> None:
        try:
            self.image.flags.writeable = True
            self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)
            self.mp_drawing.draw_landmarks(
                self.image,
                self.result.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())

            if write_angles != None:
                for label, angle in write_angles.items():
                    point = self.result.pose_landmarks.landmark[landmark_translate(
                        True, [label])[0]]
                    cv2.putText(self.image, str(round(angle, 1)), tuple(np.multiply([1 - point.x, point.y], [640, 480]).astype(
                        int)), cv2.FONT_HERSHEY_SIMPLEX, font_size, font_color, 2, cv2.LINE_AA)

            cv2.imshow(self.name, self.image)
        except:
            pass


def landmark_translate(str_to_num: bool, landmarks: list) -> list:
    landmark_dict = {
        "S1": 11, "S2": 12, "E1": 13, "E2": 14,
        "W1": 15, "W2": 16, "H1": 23, "H2": 24
    }
    inv_landmark_dict = {v: k for k, v in landmark_dict.items()}
    ret = []
    if str_to_num:
        for lm in landmarks:
            ret.append(landmark_dict[lm])
    else:
        for lm in landmarks:
            ret.append(inv_landmark_dict[lm])

    return ret
