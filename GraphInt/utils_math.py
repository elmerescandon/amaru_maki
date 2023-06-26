import numpy as np

# *********************************************
#   Funciones de Representaciones Espaciales
# *********************************************
def T2RPY(T):

    # Eje Y
    pitch = np.arctan2(-T[2,0],np.sqrt(T[2,1]**2+ T[2,2]**2))
    cos_pitch = np.cos(pitch)

    if(cos_pitch == 0):
        return
    
    # Eje Z
    roll = np.arctan2(T[1,0]/cos_pitch,T[0,0]/cos_pitch)
    
    # Eje X
    yaw = np.arctan2(T[2,1]/cos_pitch,T[2,2]/cos_pitch)

    # ZYX  
    # Z: Roll / Y: Pitch / X: Yaw
    return np.array([roll,pitch,yaw])

def rot(th, a):
    """
    Generate the Rotation matrix from an angle 'th' in 
    degrees around an 'a' axis (input as a character variable)
    """
    cos = np.cos 
    sin = np.sin
    th = np.deg2rad(th)
    if (a == 'x'):
        R = np.array([[1,       0,        0],
                      [0, cos(th), -sin(th)],
                      [0, sin(th),  cos(th)]])
    elif (a == 'y'):
        R = np.array([[cos(th),      0, sin(th)],
                      [0,      1,      0],
                      [-sin(th),     0, cos(th)]])
    elif (a == 'z'):
        R = np.array([[cos(th), -sin(th), 0],
                      [sin(th), cos(th), 0],
                      [0,      0,    1]])
    return R

def RPY2R(rpy):
    # Z: Roll / Y: Pitch / X: Yaw
    r = np.deg2rad(rpy[0])
    p = np.deg2rad(rpy[1])
    y = np.deg2rad(rpy[2])
    c = np.cos 
    s = np.sin
    R = np.array([[c(r)*c(p), c(r)*s(p)*s(y)-s(r)*c(y), c(r)*s(p)*c(y)+s(r)*s(y)],
                  [s(r)*c(p), s(r)*s(p)*s(y)+c(r)*c(y), s(r)*s(p)*c(y)-c(r)*s(y)],
                  [    -s(p),                c(p)*s(y),                c(p)*c(y)]])
    return R

def R2RPY(R):
    # Eje Y
    pitch = np.arctan2(-R[2,0],np.sqrt(R[2,1]**2+ R[2,2]**2))
    cos_pitch = np.cos(pitch)

    if(cos_pitch == 0):
        return
    
    # Eje Z
    roll = np.arctan2(R[1,0]/cos_pitch,R[0,0]/cos_pitch)
    
    # Eje X
    yaw = np.arctan2(R[2,1]/cos_pitch,R[2,2]/cos_pitch)

    # ZYX  
    # Z: Roll / Y: Pitch / X: Yaw
    return np.rad2deg(np.array([roll,pitch,yaw]))

def T2Q(R):

    trace = R[0,0] + R[1,1] + R[2,2]

    if trace>0:
        s = 0.5/np.sqrt(trace + 1.0)
        w = 0.25/s
        x = ( R[2,1] - R[1,2] )*s 
        y = ( R[0,2] - R[2,0] )*s
        z = ( R[1,0] - R[0,1] )*s
    else: 
        if (R[0,0] > R[1,1] ) and (R[0,0] > R[2,2]):
            s = 2.0*np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2] )
            w = (R[2,1] - R[1,2])/s
            x = 0.25*s
            y = (R[0,1] + R[1,0])/s 
            z = (R[0,2] + R[2,0])/s
        
        elif R[1,1] > R[2,2]:
            s = 2.0*np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            w = (R[0,2] - R[2,0])/s
            x = (R[0,1] + R[1,0])/s
            y = 0.25*s
            z = (R[1,2] + R[2,1])/s
        else: 
            s = 2.0*np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            w = (R[1,0] - R[0,1])/s
            x = (R[0,2] + R[2,0])/s
            y = (R[1,2] + R[2,1])/s
            z = 0.25*s
    
    return np.array([w,x,y,z])


def Q2T(q):
    '''
    Function that returns the Rotation Matrix from a 
    Quaternion. Check:
    http://oramosp.epizy.com/teaching/212/fund-robotica/clases/2_Representaciones_Espaciales_II.pdf
    '''
    w = q[0]
    ex = q[1]
    ey = q[2]
    ez = q[3]

    R = np.array([[2*(w**2 + ex**2)-1 ,2*(ex*ey - w*ez)   ,2*(ex*ez + w*ey)], 
                  [2*(ex*ey +  w*ez)  ,2*(w**2 + ey**2)-1 ,2*(ey*ez - w*ex),], 
                  [2*(ex*ez - w*ey)   ,2*(ey*ez + w*ex)   ,2*(w**2 + ez**2)-1]])
    

    return R

# *********************************************
#   Funciones Cinemáticas
# *********************************************
def dh(d, theta, a, alpha):
    
    T = np.array([[np.cos(theta), -np.cos(alpha)*np.sin(theta),   np.sin(alpha)*np.sin(theta), a*np.cos(theta)],
                 [np.sin(theta),   np.cos(alpha)*np.cos(theta), - np.sin(alpha)*np.cos(theta), a*np.sin(theta)],
                 [0,                             np.sin(alpha),                 np.cos(alpha),               d],
                 [0,                                         0,                             0,               1]])

    return T

def fkine_brazo(n,q,quat_bool):

    humero = 0.28 # Tamaño de brazo
    radio = 0.235 # Tamaño de antebrazo
    mano = 0.06 # Tamaño de codo hacia palma

    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    T01 = dh(0,    q1,           0,    np.pi/2)
    T12 = dh(0,    q2 + np.pi/2, 0,    np.pi/2)
    T23 = dh(humero,  q3 + np.pi/2, 0,    np.pi/2)
    T34 = dh(0,    q4 + np.pi,   0,    np.pi/2)
    T45 = dh(radio,q5 + np.pi/2, 0,    np.pi/2)
    T56 = dh(0,    q6 + np.pi/2, mano, 0)

    if (n == 1): 
        T = T01
    elif (n == 2):
        T =  T01.dot(T12)
    elif (n == 3):
        T = T01.dot(T12).dot(T23)
    elif (n == 4):
        T =  T01.dot(T12).dot(T23).dot(T34)
    elif (n == 5):
        T =  T01.dot(T12).dot(T23).dot(T34).dot(T45)
    elif (n == 6):
        T =  T01.dot(T12).dot(T23).dot(T34).dot(T45).dot(T56)
    else:
        return
    
    return T2Q(T) if quat_bool else T

def J_Qq(q_i,n,init,final,delta = 0.0001):
    """
    Cálculo del Jacobiano Numérico del cuaternion unitario calculado a partir de la
    cinemática directa del modelo de brazo
    q_i: Valores articulares iniciales
    Q_d: Orientación de quaternion deseada (medida por el sensor)
    """
    # Init normalmente comienza en 1 
    # Final normalmente hasta donde se desea
    for i in range(n):
        q_t = np.copy(q_i)

        if ( i >= init and  i < final):
            q_t[i] = q_i[i] + delta
        # q_t[i] = q_i[i] + delta


        if (i == 0):
            JT = fkine_brazo(n,q_t,1) - fkine_brazo(n,q_i,1)
        else:
            JT_tmp = fkine_brazo(n,q_t,1) - fkine_brazo(n,q_i,1)
            JT = np.vstack((JT,JT_tmp))
        
    JT = (1/delta)*JT
    J = JT.T
    
    return J
        
def invkin_DG(q_init,Q_des,n, init, final,eps = 1e-2, alpha = 0.5, max_iter = 5):
    q = q_init
    q_out = q_init[0:n]
    for i in range(max_iter):
        J = J_Qq(q,n,init,final)
        JT = J.T

        e = Q_des - fkine_brazo(n,q,1)
        tmp = JT.dot(e)
        q_out = q_out + alpha*tmp
        q[0:n] = q_out

        lin_err = np.linalg.norm(e)
        if(lin_err < eps):
            # print("Error mínimo")
            break

    # Normalize Angles
    norm_angle = q // (2*np.pi)
    # print(norm_angle)
    for i in range(norm_angle.size):
        if (abs(norm_angle[i]) > 1):
            q[i] = q[i] - norm_angle[i]*2*np.pi
            
        if (np.rad2deg(q[i]) < -180):
            q[i] = 2*np.pi + q[i]

    return q

def get_joints(data_read,q_joint):
    qw1 = data_read[0]
    qx1 = data_read[1]
    qy1 = data_read[2]
    qz1 = data_read[3]

    qw2 = data_read[4]
    qx2 = data_read[5]
    qy2 = data_read[6]
    qz2 = data_read[7]

    qw3 = data_read[8]
    qx3 = data_read[9]
    qy3 = data_read[10]
    qz3 = data_read[11]


    # Constantes
    q_joint = np.array([0.,0.,0. ,0.,0.,0.])

    q1 = np.array([qw1,qx1,qy1,qz1])
    R_0IM1 = rot(90,'z')
    R_3IM1 = rot(180,'z')
    R_IM1 = Q2T(q1)
    R_03 = R_0IM1.dot(R_IM1).dot(R_3IM1.T)


    Quat_03 = T2Q(R_03)
    q_joint_f = invkin_DG(q_joint,Quat_03,3,0,3)

    q2 = np.array([qw2,qx2,qy2,qz2])
    R_IM2 = Q2T(q2)
    R_3IM2 = rot(180,'z')
    R_5IM2 = rot(180,'x').dot(rot(90,'y'))
    R_35 = R_3IM2.dot(R_IM2).dot(R_5IM2.T)
    R_05 = R_03.dot(R_35)
    Quat_05 = T2Q(R_05)
    q_joint_f = invkin_DG(q_joint_f,Quat_05,5,0,5)


    R_6IM3 = rot(90,'x').dot(rot(90,'z'))
    R_5IM3 = rot(90,'y').dot(rot(180,'z'))
    q3 = np.array([qw3,qx3,qy3,qz3])
    R_IM3 = Q2T(q3)

    R_56 = R_5IM3.dot(R_IM3).dot(R_6IM3.T)
    R_06 = R_05.dot(R_56)
    Quat_06 = T2Q(R_06)

    q_joint_f = invkin_DG(q_joint,Quat_06,6,0,6)

    # if (any(np.isnan(q_joint))):    
    #     q_joint_f = np.array([0.,0.,0.,0.,0.,0.])
    q_joint_result = np.round(np.rad2deg(q_joint_f),2)
    # print(q_joint_result)
    return q_joint_result