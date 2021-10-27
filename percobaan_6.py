import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import PyKDL as kdl

def set_label(ax, min_scale=-10, max_scale=10):
    ax.set_xlim(min_scale, max_scale)
    ax.set_ylim(min_scale, max_scale)
    ax.set_zlim(min_scale, max_scale)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

def draw_axis(ax, scale=0.03, A=np.eye(4), style='-'):
    xaxis = np.array([[0, 0, 0, 1], [scale, 0, 0, 1]]).T
    yaxis = np.array([[0, 0, 0, 1], [0, scale, 0, 1]]).T
    zaxis = np.array([[0, 0, 0, 1], [0, 0, scale, 1]]).T
    xc = A.dot(xaxis)
    yc = A.dot(yaxis)
    zc = A.dot(zaxis) 
    ax.plot(xc[0,:], xc[1,:], xc[2,:], 'r' + style)
    ax.plot(yc[0,:], yc[1,:], yc[2,:], 'g' + style)
    ax.plot(zc[0,:], zc[1,:], zc[2,:], 'b' + style)
    #print '===='
    #print xc
def RX(alpha):
    return np.array([[1, 0, 0], 
                     [0, math.cos(alpha), -math.sin(alpha)], 
                     [0, math.sin(alpha), math.cos(alpha)]])   

def RY(delta):
    return np.array([[math.cos(delta), 0, math.sin(delta)], 
                     [0, 1, 0], 
                     [-math.sin(delta), 0, math.cos(delta)]])

def RZ(theta):
    return np.array([[math.cos(theta), -math.sin(theta), 0], 
                     [math.sin(theta), math.cos(theta), 0], 
                     [0, 0, 1]])

def TF(rot_axis=None, q=0, dx=0, dy=0, dz=0):
    if rot_axis == 'x':
        R = RX(q)
    elif rot_axis == 'y':
        R = RY(q)
    elif rot_axis == 'z':
        R = RZ(q)
    elif rot_axis == None:
        R = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])
    
    T = np.array([[R[0,0], R[0,1], R[0,2], dx],
                  [R[1,0], R[1,1], R[1,2], dy],
                  [R[2,0], R[2,1], R[2,2], dz],
                  [0, 0, 0, 1]])
    return T

def draw_links(ax, origin_frame=np.eye(4), target_frame=np.eye(4)):
    x = [origin_frame[0,3], target_frame[0,3]]
    y = [origin_frame[1,3], target_frame[1,3]]
    z = [origin_frame[2,3], target_frame[2,3]]
    ax.plot(x, y, z, 'k')

def pose_error(f_target, f_result):
    f_diff = f_target.Inverse() * f_result
    #print (f_target.Inverse(), f_diff)

    [dx, dy, dz] = f_diff.p
    #print ([dx, dy, dz])
    [drz, dry, drx] = f_diff.M.GetEulerZYX()
    #print ([drx, dry, drz])
    error = np.sqrt(dx**2 + dy**2 + dz**2 + drx**2 + dry**2 + drz**2)
    error_list = [dx, dy, dz, drx, dry, drz]
    return error, error_list

def main():
    # Konfigurasi link
    CROTCH_TO_HIP = 0.105    # Jarak croth ke hip
    UPPER_HIP = 0.0925       # Jarak hip yaw ke hip roll pitch
    HIP_TO_KNEE = 0.275      # Panjang link upper leg
    KNEE_TO_ANKLE = 0.275    # Panjang link lower leg
    ANKLE_TO_SOLE = 0.0765   # Jarak ankle ke sole
    UP_KNEE_TO_DOWN_KNEE = 0 #jarak joint pada lutut
    
    A = HIP_TO_KNEE
    B = KNEE_TO_ANKLE
    D = CROTCH_TO_HIP

    # Input target
    x_from_base = 0
    y_from_base = 0.105
    z_from_base = -0.652 + 0.1
    yaw = np.radians(0)

    # frame target
    f_target = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw), kdl.Vector(x_from_base, y_from_base, z_from_base - 0.1))
    
    # Jarak target ke hip
    x_from_hip = (x_from_base - 0)
    y_from_hip = (y_from_base - CROTCH_TO_HIP)
    z_from_hip = (z_from_base + UPPER_HIP + ANKLE_TO_SOLE)

    # jarak target ke hip dengan yaw
    xa = x_from_hip
    ya_1 = xa*np.tan(yaw)
    xb_1 = xa / np.cos(yaw)
    beta = np.radians(90) - yaw
    ya_2 = (y_from_hip-ya_1)
    yb = ya_2 * np.sin(beta)
    xb_2 = yb / np.tan(beta) #!90
    xb = xb_1 + xb_2
    #print ya_1, ya_2, yb, xb_1, xb_2, xb, beta

    x_from_hip = xb
    y_from_hip = yb

    z_from_hip = z_from_hip
    z_from_hip2 = z_from_hip - 0.16
    #print xb, yb, z_from_hip
    # Inverse Kinematic
    # Mencari jarak Target ke Hip (C)
    C = np.sqrt(x_from_hip**2+y_from_hip**2+z_from_hip**2)
    print("C =", C)

    # Mencari sudut Knee
    q4 = -np.arccos(((A**2+B**2)-C**2)/(2*A*B))
    pbwh = (C-UP_KNEE_TO_DOWN_KNEE)/2
    q41 = -(np.radians(180)-(np.arcsin((pbwh / A)) + np.radians(90)))
    q42 = q41
    print("q4 = ",np.degrees(q41))
    qKnee = q4+np.radians(180)
    qKneeDown = q41
    qKneeUp = q41
    # Mencari sudut ankle pitch dan hip pitch
    q3 = qKnee/2
    print("q3 = ",np.degrees(q3))
    qx = np.arcsin(x_from_hip/C)
    print("qx =", np.degrees(qx))
    qHipPitch = -(q3+qx)
    
    q5 = q3
    qz = (np.sqrt(y_from_hip**2+z_from_hip**2))
    print("qz :", np.degrees(qz))
    qAnklePitch = ((np.radians(180)-(np.arctan2(x_from_hip, np.sign(z_from_hip)*qz))) - q3)
    qHipYaw = yaw
    qHipRoll = np.arctan2(y_from_hip, np.sign(z_from_hip2)*z_from_hip2)
    qAnkleRoll = -qHipRoll
    qKneeDown = qAnklePitch
    qKneeUp = qHipPitch


    fig = plt.figure() # Deklarasi matplot
    ax = fig.add_subplot(111, projection='3d')
    print("Q hip Yaw 	: ", np.degrees(qHipYaw))
    print("Q hip Roll 	: ", qHipRoll)
    print("Q hip Pitch 	: ",qHipPitch)
    print("Q Knee UP	: ",np.degrees(qKneeUp))
    print("Q Knee Down 	: ",np.degrees(qKneeDown))
    print("Q ankle Pitch 	: ", qAnklePitch)
    print("Q ankle Roll 	: ", qAnkleRoll)
    

    angle = [-qHipYaw, qHipRoll, qHipPitch, qKneeUp, qKneeDown, -qAnklePitch, -qAnkleRoll]
    
    # Forward kinematic
    base = TF()
    hip_yaw_from_base = TF(rot_axis='z', q=qHipYaw, dy=D)
    hip_yaw = base.dot(hip_yaw_from_base) #R01
    hip_roll_from_hip_yaw = TF(rot_axis='x', q=qHipRoll, dz=-UPPER_HIP)
    hip_roll = hip_yaw.dot(hip_roll_from_hip_yaw) #R12
    hip_pitch_from_hip_roll = TF(rot_axis='y', q=qHipPitch)
    hip_pitch = hip_roll.dot(hip_pitch_from_hip_roll) #R23
    hip = hip_pitch # hip frame
    knee_from_hip = TF(rot_axis='y', q=-qKneeUp, dz=-A)
    knee_up = hip.dot(knee_from_hip) #R34
    kneeDown_from_up = TF(rot_axis='y', q=-qKneeDown, dz=-0.1)
    knee_Down = knee_up.dot(kneeDown_from_up) 
    ankle_pitch_from_knee = TF(rot_axis='y', q=qAnklePitch, dz=-B)
    ankle_pitch = knee_Down.dot(ankle_pitch_from_knee) #R45
    ankle_roll_from_ankle_pitch = TF(rot_axis='x', q=qAnkleRoll)
    ankle_roll = ankle_pitch.dot(ankle_roll_from_ankle_pitch) #R56
    ankle = ankle_roll
    sole_from_ankle = TF(dz=-ANKLE_TO_SOLE)
    sole = ankle.dot(sole_from_ankle) # sole frame R67
    '''
    base = TF()
    hip_yaw_from_base = TF(rot_axis='z', q=0, dy=D)
    hip_yaw = base.dot(hip_yaw_from_base) #R01
    hip_roll_from_hip_yaw = TF(rot_axis='x', q=0.6, dz=-UPPER_HIP)
    hip_roll = hip_yaw.dot(hip_roll_from_hip_yaw) #R12
    hip_pitch_from_hip_roll = TF(rot_axis='y', q=-0.8)
    hip_pitch = hip_roll.dot(hip_pitch_from_hip_roll) #R23
    hip = hip_pitch # hip frame
    knee_from_hip = TF(rot_axis='y', q=0.8, dz=-A)
    knee_up = hip.dot(knee_from_hip) #R34
    kneeDown_from_up = TF(rot_axis='y', q=0.8, dz=-0.16)
    knee_Down = knee_up.dot(kneeDown_from_up) 
    ankle_pitch_from_knee = TF(rot_axis='y', q=-0.8, dz=-B)
    ankle_pitch = knee_Down.dot(ankle_pitch_from_knee) #R45
    ankle_roll_from_ankle_pitch = TF(rot_axis='x', q=-0.6)
    ankle_roll = ankle_pitch.dot(ankle_roll_from_ankle_pitch) #R56
    ankle = ankle_roll
    sole_from_ankle = TF(dz=-ANKLE_TO_SOLE)
    sole = ankle.dot(sole_from_ankle) # sole frame R67
    '''

    draw_axis(ax, A=base)
    draw_links(ax, origin_frame=base, target_frame=hip_yaw)
    draw_axis(ax, A=hip_yaw)
    draw_links(ax, origin_frame=hip_yaw, target_frame=hip)
    draw_axis(ax, A=hip)
    draw_links(ax, origin_frame=hip, target_frame=knee_up)
    draw_axis(ax, A=knee_up)
    draw_links(ax, origin_frame=knee_up, target_frame=knee_Down)
    draw_axis(ax, A=knee_Down)
    draw_links(ax, origin_frame=knee_Down, target_frame=ankle)
    draw_axis(ax, A=ankle)
    draw_links(ax, origin_frame=ankle, target_frame=sole)
    draw_axis(ax, A=sole)

    f_result = kdl.Frame(kdl.Rotation(sole[0,0], sole[0,1], sole[0,2],
                                    sole[1,0], sole[1,1], sole[1,2],
                                    sole[2,0], sole[2,1], sole[2,2]),
                        kdl.Vector(sole[0,3], sole[1,3], sole[2,3]))
    rot = f_result.M
    rpy = []
    rpy = rot.GetEulerZYX()
    #print("RPY = ",rpy) 
    error,errorList = pose_error(f_target,f_result)
    
    print("Input Position")
    print("x :", x_from_base)
    print("y :", y_from_base)
    print("z :", z_from_base - 0.1)
    print("yaw :", np.degrees(yaw))
    print("Solution")
    print("x :", sole[0,3])
    print("y :", sole[1,3])
    print("z :", sole[2,3])
    #print("F result = ", f_result)
    print("ERROR :", error)
    print("ERROR LIST :", errorList)
    #ax.auto_scale_xyz([-4, 4], [-2, 3], [3, -9.5])
    #set_label(ax)
    plt.show()
if __name__ == "__main__":
    main()      
