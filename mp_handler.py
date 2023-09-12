import cv2
import mediapipe as mp


class PoseGetter:
    def __init__(self, camera_num, name, wanted_points):
        self.cap = cv2.VideoCapture(camera_num)
        self.name = name
        self.wanted_points = wanted_points
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self

    def is_open(self):
        return self.cap.isOpened()

    def run_cycle(self, show_vid):
        success, image = self.cap.read()
        if not success:
            print("camera does not work")
            return None

        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        result = self.pose.process(image)
        landmarks = result.pose_landmarks

        ret = []
        if landmarks != None:
            for k in self.wanted_points:
                lm = landmarks.landmark[k]
                ret.append([lm.x, lm.y])

        if show_vid:
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            self.mp_drawing.draw_landmarks(
                image,
                result.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())

        return ret, image
