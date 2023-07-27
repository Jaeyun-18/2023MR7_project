import cv2
import mediapipe as mp
import numpy as np
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose


def calculate_sh_joint_1(a,b):
    a = np.array(a) #left shoulder / sidecame
    b = np.array(b) #left elbow / sidecame

    radian = np.arctan(np.abs(a[1]-b[1]) / np.abs(a[0]-b[0]))
    angle = np.abs(radian)*180.0/np.pi
    
    if b[0] > a[0]:
        return angle - 90.0
    else:   
        return 90.0 - angle



def calculate_sh_joint_2(a,b,c,basic_length):
    a = np.array(a) # First
    b = np.array(b) # MId
    c = np.array(c) # End

    #180도 회전 좌측 각도 계산

    x_arm_1 = np.abs(c[0]-b[0]) # x축에 내린 윗팔의 정사영 길이
    theta = 90.0 - np.abs(np.arccos(x_arm_1/basic_length)*180.0/np.pi)
    
    y_left_shoulder = b[1]
    UP = 0 #위, 아래 판단 bool 변수

    if c[1] < y_left_shoulder:
        UP = 1
    elif c[1] >= y_left_shoulder:
        UP = 0
        
    return theta, UP

cap  = cv2.VideoCapture(0) #front
cap2  = cv2.VideoCapture(2) #side

## Setup mediapipe instance
# 얼마의 정확도로 추정할지 선정
with mp_pose.Pose(min_detection_confidence = 0.5, min_tracking_confidence = 0.5) as pose:
    while cap.isOpened():
        ret, frame = cap.read()
        ret2, frame2 = cap2.read()
        
        # Detect stuff and render
        # Recolor image to RGB
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        image2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2RGB)
        image2.flags.writeable = False
        
        # Make detection
        results = pose.process(image)

        results2 = pose.process(image2)
        
        # Recolor image to BGR
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        image2.flags.writeable = True
        image2 = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
        
        # Extract landmarks
        try:
            landmarks = results.pose_landmarks.landmark
            landmarks2 = results2.pose_landmarks.landmark
                
            # Get coordinates
            shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            elbow_front = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
            left_hip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y]
            
            shoulder1 = [landmarks2[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmarks2[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            shoulder2 = [landmarks2[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,landmarks2[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            elbow_side = [landmarks2[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,landmarks2[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            
            # Calculate angle
            angle1, UP = calculate_sh_joint_2(left_hip, shoulder, elbow_front, 0.2)  
            angle2 = calculate_sh_joint_1(shoulder1, elbow_side)
            
            #Visualize angle on frame
            cv2.putText(image, str(angle1)+" "+str(UP),
                        tuple(np.multiply(shoulder, [640, 480]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA
                       ) 
            
            cv2.putText(image2, str(angle2),
                           tuple(np.multiply(shoulder1, [640, 480]).astype(int)),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA
                       ) 
        
        except:
            pass
        
        # Render detections
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                 mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                                 # joint 크기 및 색깔 설정
                                 mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                 # connection(bone) 크기 및 색깔 설정
                                 )
        

        mp_drawing.draw_landmarks(image2, results2.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                 mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                                 # joint 크기 및 색깔 설정
                                 mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                 # connection(bone) 크기 및 색깔 설정
                                 )
        
        cv2.imshow("Mediapipe Video(front)", image)
        cv2.imshow("Mediapipe Video(side)", image2)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap.release()
    cap2.release()
    cv2.destroyAllWindows()