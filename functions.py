import numpy as np

def calcualte_left_Shoudler_angle_coor(right_shoulder_coor, left_shoulder_coor, left_elbow_coor, left_hip_coor):
    
    Shoulder_vec = np.array([left_shoulder_coor[0] - right_shoulder_coor[0],
                             left_shoulder_coor[1] - right_shoulder_coor[1],
                             left_shoulder_coor[2] - right_shoulder_coor[2]])
    
    Upper_arm_vec = np.array([left_elbow_coor[0] - left_shoulder_coor[0], 
                              left_elbow_coor[1] - left_shoulder_coor[1], 
                              left_elbow_coor[2] - left_shoulder_coor[2]])
    
    Body_vec = np.array([left_shoulder_coor[0] - left_hip_coor[0],
                         left_shoulder_coor[1] - left_hip_coor[1],
                         left_shoulder_coor[2] - left_hip_coor[2]])

    Shoulder_angle_1, Shoulder_angle_2= cal_shoulder_anlge_vec(Shoulder_vec, Upper_arm_vec, Body_vec)

    return Shoulder_angle_1, Shoulder_angle_2


def calculate_Left_Shoulder_angle(S1_F, S2_F, E1_F, S1_L, E1_L, S2_L):
    #Shoulder vector
    vec_x = S2_F[0] - S1_F[0]
    vec_y = S2_F[1] - S1_F[1]
    vec_z = S2_L[0] - S1_L[0]
    Shoulder_vec = np.array([vec_x, vec_y, vec_z])

    #Upper_arm_vector
    Shoulder_x = S1_F[0]
    Shoulder_y = S1_F[1]
    Shoulder_z = S1_L[0]
    Elbow_x = E1_F[0]
    Elbow_y = E1_F[1]
    Elbow_z = E1_L[0]
    Upper_arm_vec = np.array([Elbow_x-Shoulder_x, Elbow_y-Shoulder_y, Elbow_z-Shoulder_z])

    Shoulder_angle_1, Shoulder_angle_2= cal_shoulder_anlge_vec(Shoulder_vec, Upper_arm_vec)

    return Shoulder_angle_1, Shoulder_angle_2


def calculate_left_Elbow_angle_coor(right_shoulder_coor, left_shoulder_coor, left_elbow_coor, left_wrist_coor):
    Shoulder_vec = np.array([left_shoulder_coor[0] - right_shoulder_coor[0],
                             left_shoulder_coor[1] - right_shoulder_coor[1],
                             left_shoulder_coor[2] - right_shoulder_coor[2]])
    
    Upper_arm_vec = np.array([left_elbow_coor[0] - left_shoulder_coor[0], 
                              left_elbow_coor[1] - left_shoulder_coor[1], 
                              left_elbow_coor[2] - left_shoulder_coor[2]])
    
    Lower_arm_vec = np.array([left_wrist_coor[0] - left_elbow_coor[0],
                              left_wrist_coor[1] - left_elbow_coor[1],
                              left_wrist_coor[2] - left_elbow_coor[2]])
    
    Elbow_angle_1, Elbow_angle_2 = cal_elbow_angle_vec(Shoulder_vec, Upper_arm_vec, Lower_arm_vec)

    return Elbow_angle_1, Elbow_angle_2


def calculate_Left_Elbow_angle(S1_F,S1_L,E1_F,W1_F,H1_F,E1_L,W1_L,H1_L,S2_F,S2_L,shoulder_length): #아랫팔 벡터용 함수
    p = shoulder_length/(S1_F[0]-S2_F[0])
    #FRONT cam
    E1_F_x = p*(E1_F[0]-S1_F[0]) #왼쪽 팔꿈치 x자표
    E1_F_y = p*(E1_F[1]-S1_F[1]) #왼쪽 팔꿈치 y좌표
    W1_F_x = p*(W1_F[0]-S1_F[0]) #왼쪽 팔목 x좌표
    W1_F_y = p*(W1_F[1]-S1_F[1]) #왼쪽 팔목 y좌표

    #LEFT cam
    q = (S1_F[1]-H1_F[1])/(S1_L[1]-H1_L[1])*p
    E1_L_z = q*(E1_F[0]-S1_L[0]) #왼쪽 팔꿈치 z좌표
    W1_L_z = q*(W1_L[0]-S1_L[0]) #왼쪽 팔목 z좌표

    X_Sh = p*(S2_F[0]-S1_F[0])
    Y_Sh = p*(S2_F[1]-S1_F[1])
    Z_Sh = q*(S2_L[0]-S1_L[0])
    Shoulder_vec = np.array([X_Sh,Y_Sh,Z_Sh])

    X_Up = E1_F_x
    Y_Up = E1_F_y
    Z_Up = E1_L_z
    Upper_arm_vec = np.array([X_Up,Y_Up,Z_Up])

    X_Low = W1_F_x - E1_F_x
    Y_Low = W1_F_y - E1_F_y
    Z_Low = W1_L_z - E1_L_z
    Lower_arm_vec = np.array([X_Low,Y_Low,Z_Low])

    Elbow_angle_1, Elbow_angle_2 = cal_elbow_angle_vec(Shoulder_vec, Upper_arm_vec, Lower_arm_vec)

    return Elbow_angle_1, Elbow_angle_2    


def calculate_right_Shoulder_anlge_coor(right_shoulder_coor, left_shoulder_coor, right_elbow_coor, right_hip_coor):
    Shoulder_vec = np.array([right_shoulder_coor[0] - left_shoulder_coor[0],
                             right_shoulder_coor[1] - left_shoulder_coor[1],
                             right_shoulder_coor[2] - left_shoulder_coor[2]])
    
    Upper_arm_vec = np.array([right_elbow_coor[0] - right_shoulder_coor[0], 
                              right_elbow_coor[1] - right_shoulder_coor[1], 
                              right_elbow_coor[2] - right_shoulder_coor[2]])
    
    Body_vec = np.array([right_shoulder_coor[0] - right_hip_coor[0],
                         right_shoulder_coor[1] - right_hip_coor[1],
                         right_shoulder_coor[2] - right_hip_coor[2]])

    Shoulder_angle_1, Shoulder_angle_2= cal_shoulder_anlge_vec(Shoulder_vec, Upper_arm_vec, Body_vec)
    return Shoulder_angle_1, Shoulder_angle_2


def calculate_right_Elbow_angle(right_shoulder_coor, left_shoulder_coor, right_elbow_coor, right_wrist_coor):
    Shoulder_vec = np.array([right_shoulder_coor[0] - left_shoulder_coor[0],
                             right_shoulder_coor[1] - left_shoulder_coor[1],
                             right_shoulder_coor[2] - left_shoulder_coor[2]])
    
    Upper_arm_vec = np.array([right_elbow_coor[0] - right_shoulder_coor[0], 
                              right_elbow_coor[1] - right_shoulder_coor[1], 
                              right_elbow_coor[2] - right_shoulder_coor[2]])
    
    Lower_arm_vec = np.array([right_wrist_coor[0] - right_elbow_coor[0],
                              right_wrist_coor[1] - right_elbow_coor[1],
                              right_wrist_coor[2] - right_elbow_coor[2]])
    
    Elbow_angle_1, Elbow_angle_2 = cal_elbow_angle_vec(Shoulder_vec, Upper_arm_vec, Lower_arm_vec)

    return Elbow_angle_1, Elbow_angle_2


def cal_shoulder_anlge_vec(Shoulder_vec, Upper_arm_vec, Body_vec):
    #첫번째 각도
    Shoulder_angle_1_semi = np.arccos(np.dot(Upper_arm_vec,Shoulder_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(Shoulder_vec)))
    Shoulder_angle_1 = (Shoulder_angle_1_semi/np.pi)*180.0

    #두번째 각도 / 정면으로 뻗은걸 0도로 기준
    Body_cross_vec = np.cross(Shoulder_vec, Body_vec)/(np.linalg.norm(Shoulder_vec)*np.linalg.norm(Body_vec))
    Shoulder_cross_vec = np.cross(Shoulder_vec, Upper_arm_vec)/(np.linalg.norm(Shoulder_vec)*np.linalg.norm(Body_vec))
    rev_Shoulder_cross_vec = (-1)*Shoulder_cross_vec
    Shoulder_angle_2_semi = np.arccos(np.dot(Body_cross_vec,Shoulder_cross_vec)/np.linalg.norm(Body_cross_vec)*np.linalg.norm(Shoulder_cross_vec))
    Shoulder_angle_2_semi_rev = np.arccos(np.dot(rev_Shoulder_cross_vec,Shoulder_cross_vec)/np.linalg.norm(rev_Shoulder_cross_vec)*np.linalg.norm(Shoulder_cross_vec))

    Shoulder_angle_2 = (Shoulder_angle_2_semi/np.pi)*180.0

    return Shoulder_angle_1, Shoulder_angle_2


def cal_elbow_angle_vec(Shoulder_vec, Upper_arm_vec, Lower_arm_vec):
    Elbow_angle_1_semi = np.arccos(np.dot(Upper_arm_vec,Lower_arm_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(Lower_arm_vec)))
    Elbow_angle_1 = 180.0-(Elbow_angle_1_semi/np.pi)*180.0

    Upper_cross_vec = np.cross(Shoulder_vec,Upper_arm_vec)/(np.linalg.norm(Shoulder_vec)*np.linalg.norm(Upper_arm_vec))
    Lower_cross_vec = np.cross(Upper_arm_vec,Lower_arm_vec)/(np.linalg.norm(Upper_arm_vec)*np.linalg.norm(Lower_arm_vec))

    Elbow_angle_2_semi = np.arccos(np.dot(Upper_cross_vec,Lower_cross_vec)/(np.linalg.norm(Upper_cross_vec)*np.linalg.norm(Lower_cross_vec)))
    Elbow_angle_2 = (Elbow_angle_2_semi/np.pi)*180.0
 
    return Elbow_angle_1, Elbow_angle_2


"""
def trans_motor_anlge():
    Sign = 1.0

    if Elbow_z > Shoulder_z:
        Sign = +1.0  
    else:   
        Sign = -1.0
"""