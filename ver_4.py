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

def calculate_Elbow_angle(S1_F,S1_L,E1_F,W1_F,H1_F,E1_L,W1_L,H1_L,S2_F,shoulder_length): #아랫팔 벡터용 함수
    p = shoulder_length/(S1_F[0]-S2_F[0])
    #FRONT cam
    E1_F_x = p*(E1_F[0]-S1_F[0]) #왼쪽 팔꿈치 x자표
    E1_F_y = p*(E1_F[1]-S1_F[1]) #왼쪽 팔꿈치 y좌표
    W1_F_x = p*(W1_F[0]-S1_F[0]) #왼쪽 팔목 x좌표
    W1_F_y = p*(W1_F[1]-S1_F[1]) #왼쪽 팔목 y좌표

    #LEFT cam
    q = (S1_F[1]-H1_F[1])/(S1_L[1]-H1_L[1])
    E1_L_z = q*(E1_F[0]-S1_L[0]) #왼쪽 팔꿈치 z좌표
    W1_L_z = q*(W1_L[0]-S1_L[0]) #왼쪽 팔목 z좌표

    X_Sh = p*(S2_F[0]-S1_F[0])
    Y_Sh = 0.0
    Z_Sh = 0.0
    Shoulder_vec = np.array([X_Sh,Y_Sh,Z_Sh])

    X_Up = E1_F_x
    Y_Up = E1_F_y
    Z_Up = E1_L_z
    Upper_arm_vec = np.array([X_Up,Y_Up,Z_Up])

    X_Low = W1_F_x - E1_F_x
    Y_Low = W1_F_y - E1_F_y
    Z_Low = W1_L_z - E1_L_z
    Lower_arm_vec = np.array([X_Low,Y_Low,Z_Low])

    Elbow_angle_1_semi = np.arccos(np.dot(Upper_arm_vec,Lower_arm_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(Lower_arm_vec)))
    Elbow_angle_1 = 180.0-(Elbow_angle_1_semi/np.pi)*180.0

    Upper_cross_vec = np.cross(Shoulder_vec,Upper_arm_vec)/(np.linalg.norm(Shoulder_vec)*np.linalg.norm(Upper_arm_vec))
    Lower_cross_vec = np.cross(Upper_arm_vec,Lower_arm_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(Lower_arm_vec))

    Elbow_angle_2_semi = np.arccos(np.dot(Upper_cross_vec,Lower_cross_vec)/(np.linalg.norm(Upper_cross_vec)*np.linalg.norm(Lower_cross_vec)))
    Elbow_angle_2 = (Elbow_angle_2_semi/np.pi)*180.0

    print(Elbow_angle_1)
    print(Elbow_angle_2)

    return Elbow_angle_1, Elbow_angle_2


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
        
        # image_R = cv2.cvtColor(frame_L, cv2.COLOR_BGR2RGB)
        # image_R.flags.writeable = False

        # Make detection
        result_F = pose.process(image_F)

        result_L = pose.process(image_L)

        # result_R = pose.process(image_R)
        
        # Recolor image_F to BGR
        image_F.flags.writeable = True
        image_F = cv2.cvtColor(image_F, cv2.COLOR_BGR2RGB)

        image_L.flags.writeable = True
        image_L = cv2.cvtColor(image_L, cv2.COLOR_BGR2RGB)

        # image_R.flags.writeable = True
        # image_R = cv2.cvtColor(image_R, cv2.COLOR_BGR2RGB)
        
        # Extract landmark_F
        try:
            landmark_F = result_F.pose_landmarks.landmark
            landmark_L = result_L.pose_landmarks.landmark
            # landmark_R = result_R.pose_landmarks.landmark
                
            # Get coordinates
            S1_F = [landmark_F[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmark_F[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            S2_F = [landmark_F[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,landmark_F[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            E1_F = [landmark_F[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,landmark_F[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            E2_F = [landmark_F[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,landmark_F[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
            W1_F = [landmark_F[mp_pose.PoseLandmark.LEFT_WRIST.value].x,landmark_F[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
            W2_F = [landmark_F[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,landmark_F[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]
            H1_F = [landmark_F[mp_pose.PoseLandmark.LEFT_HIP.value].x,landmark_F[mp_pose.PoseLandmark.LEFT_HIP.value].y]
            H2_F = [landmark_F[mp_pose.PoseLandmark.RIGHT_HIP.value].x,landmark_F[mp_pose.PoseLandmark.RIGHT_HIP.value].y]
            
            S1_L = [landmark_L[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmark_L[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            S2_L = [landmark_L[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,landmark_L[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            E1_L = [landmark_L[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,landmark_L[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
            W1_L = [landmark_L[mp_pose.PoseLandmark.LEFT_WRIST.value].x,landmark_L[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
            
            # S2_R = [landmark_R[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,landmark_R[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
            # S1_R = [landmark_R[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,landmark_R[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
            # E2_R = [landmark_R[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,landmark_R[mp_pose.PoseLandmark.RIGHTBOW.value].y]
            # W2_R = [landmark_R[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,landmark_R[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]

            # Calculate angle
            angle_front_L, UP = calculate_S1_front(H1_F, S1_F, E1_F, 0.2)  
            angle_side_L = calculate_S1_side(S1_L, E1_L)
            angle_frontside_L_2 = calculate_E1(S1_F,E1_F,W1_F,S1_L,E1_L,W1_L) #아랫팔 벡터 구할 때 사용.
            
            #Visualize angle on frame_F
            cv2.putText(image_F, str(Elbow_angle_1)+" "+str(Elbow_angle_2),
                        tuple(np.multiply(S1_F, [640, 480]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA
                       ) 
            
            # cv2.putText(image_L, str(angle_side_L),
            #                tuple(np.multiply(S1_L, [640, 480]).astype(int)),
            #                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA
            #            ) 
            
        
        except Exception as e:
            print(e)
        
        # Render detections
        mp_drawing.draw_landmarks(image_F, result_F.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                 mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                                 # joint 크기 및 색깔 설정
                                 mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                 # connection(bone) 크기 및 색깔 설정
                                 )
        

        mp_drawing.draw_landmarks(image_L, result_L.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                 mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
                                 # joint 크기 및 색깔 설정
                                 mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
                                 # connection(bone) 크기 및 색깔 설정
                                 )
        
        # mp_drawing.draw_landmark_R(image_R, result_R.pose_landmark_F, mp_pose.POSE_CONNECTIONS,
        #                          mp_drawing.DrawingSpec(color=(245,117,66), thickness=2, circle_radius=2),
        #                          # joint 크기 및 색깔 설정
        #                          mp_drawing.DrawingSpec(color=(245,66,230), thickness=2, circle_radius=2)
        #                          # connection(bone) 크기 및 색깔 설정
        #                          )
        
        cv2.imshow("Mediapipe Video(front)", image_F)
        cv2.imshow("Mediapipe Video(left)", image_L)
        #cv2.imshow("Mediapipe Video(right)",image_R)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    cap_F.release()
    cap_L.release()
    # cap_R.release()
    cv2.destroyAllWindows()