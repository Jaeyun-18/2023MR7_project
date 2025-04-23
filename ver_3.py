import cv2
import mediapipe as mp
import numpy as np
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

def calculate_S1_side(S1_L, E1_L): #좌측면, 원
    S1_L = np.array(S1_L) #left shoulder / sidecame
    E1_L = np.array(E1_L) #left elbow / sidecame

    radian = np.arctan(np.abs(S1_L[1]-E1_L[1]) / np.abs(S1_L[0]-E1_L[0]))
    angle_L = np.abs(radian)*180.0/np.pi
    
    if E1_L[0] > S1_L[0]:
        return angle_L - 90.0
    else:   
        return 90.0 - angle_L
    
def calculate_S1_front(H1_F, S1_F, E1_F, basic_length): #정면, 좌측 반원
    H1_F = np.array(H1_F) # First
    S1_F = np.array(S1_F) # Mid
    E1_F = np.array(E1_F) # End

    x_arm_1 = np.abs(E1_F[0]-S1_F[0]) # x축에 내린 윗팔의 정사영 길이
    angle_front = 90.0 - np.abs(np.arccos(x_arm_1/basic_length)*180.0/np.pi)
    
    y_S1 = S1_F[1]
    UP = 0 #위, 아래 판단 bool 변수

    if E1_F[1] < y_S1:
        UP = 1
    elif E1_F[1] >= y_S1:
        UP = 0
        
    return angle_front, UP

cap_F  = cv2.VideoCapture(1) #front
cap_L = cv2.VideoCapture(0) #left
#cap_R = cv3.VideoCapture(#) #right

with mp_pose.Pose(min_detection_confidence = 0.5, min_tracking_confidence = 0.5) as pose:
    while cap_F.isOpened():
        ret_F, frame_F = cap_F.read()
        ret_L, frame_L = cap_L.read()
        
        # Detect stuff and render
        # Recolor image_F to RGB
        image_F = cv2.cvtColor(frame_F, cv2.COLOR_BGR2RGB)
        image_F.flags.writeable = False

        image_L = cv2.cvtColor(frame_L, cv2.COLOR_BGR2RGB)
        image_L.flags.writeable = False
        
        # Make detection
        result_F = pose.process(image_F)

        result_L = pose.process(image_L)
        
        # Recolor image_F to BGR
        image_F.flags.writeable = True
        image_F = cv2.cvtColor(image_F, cv2.COLOR_BGR2RGB)

        image_L.flags.writeable = True
        image_L = cv2.cvtColor(image_L, cv2.COLOR_BGR2RGB)
        
        # Extract landmarks_F
        try:
            landmarks_F = result_F.pose_landmarks_F.landmark
            landmark_L = result_L.pose_landmarks_F.landmark
                
            # Get coordinates
            S1_F = [landmarks_F[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmarks_F[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            S2_F = [landmarks_F[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,landmarks_F[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            E1_F = [landmarks_F[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,landmarks_F[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            E2_F = [landmarks_F[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,landmarks_F[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
            W1_F = [landmarks_F[mp_pose.PoseLandmark.LEFT_WRIST.value].x,landmarks_F[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
            W2_F = [landmarks_F[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,landmarks_F[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
            H1_F = [landmarks_F[mp_pose.PoseLandmark.LEFT_HIP.value].x,landmarks_F[mp_pose.PoseLandmark.LEFT_HIP.value].y]
            H2_F = [landmarks_F[mp_pose.PoseLandmark.RIGHT_HIP.value].x,landmarks_F[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
            
            S1_L = [landmark_L[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmark_L[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            S2_L = [landmark_L[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,landmark_L[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            E1_L = [landmark_L[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,landmark_L[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            
            # Calculate angle
            angle_front_L, UP = calculate_S1_front(H1_F, S1_F, E1_F, 0.2)  
            angle_side_L = calculate_S1_side(S1_L, E1_L)
            
            #Visualize angle on frame_F
            cv2.putText(image_F, str(angle_front_L)+" "+str(UP),
                        tuple(np.multiply(S1_F, [640, 480]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA
                       ) 
            
            cv2.putText(image_L, str(angle_side_L),
                           tuple(np.multiply(S1_L, [640, 480]).astype(int)),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA
                       ) 
        
        except:
            pass
        
        # Render detections
        mp_drawing.draw_landmarks_F(image_F, result_F.pose_landmarks_F, mp_pose.POSE_CONNECTIONS,
                                 mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                                 # joint 크기 및 색깔 설정
                                 mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                 # connection(bone) 크기 및 색깔 설정
                                 )
        

        mp_drawing.draw_landmarks_F(image_L, result_L.pose_landmarks_F, mp_pose.POSE_CONNECTIONS,
                                 mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                                 # joint 크기 및 색깔 설정
                                 mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                 # connection(bone) 크기 및 색깔 설정
                                 )
        
        cv2.imshow("Mediapipe Video(front)", image_F)
        cv2.imshow("Mediapipe Video(side)", image_L)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap_F.release()
    cap_L.release()
    cv2.destroyAllWindows()