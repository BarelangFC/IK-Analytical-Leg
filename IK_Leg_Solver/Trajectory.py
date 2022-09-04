#!/usr/bin/env python3

import math
import numpy as np
from numpy import array as a, degrees
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import PyKDL as kdl
from time import time, sleep
from numpy.lib.function_base import angle
import rospy
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

def set_label(ax, min_scale=-10, max_scale=10):
    ax.set_xlim(min_scale, max_scale)
    ax.set_ylim(min_scale, max_scale)
    ax.set_zlim(min_scale, max_scale)
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

def draw_axis(ax, scale=0.5, A=np.eye(4), style='-'):
    xaxis = np.array([[0, 0, 0, 1], [scale, 0, 0, 1]]).T
    yaxis = np.array([[0, 0, 0, 1], [0, scale, 0, 1]]).T
    zaxis = np.array([[0, 0, 0, 1], [0, 0, scale, 1]]).T
    xc = A.dot(xaxis)
    yc = A.dot(yaxis)
    zc = A.dot(zaxis) 
    ax.plot(xc[0,:], xc[1,:], xc[2,:], 'r' + style)
    ax.plot(yc[0,:], yc[1,:], yc[2,:], 'g' + style)
    ax.plot(zc[0,:], zc[1,:], zc[2,:], 'b' + style)
   
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
    [dx, dy, dz] = f_diff.p
    [drz, dry, drx] = f_diff.M.GetEulerZYX()
    error = np.sqrt(dx**2 + dy**2 + dz**2 + drx**2 + dry**2 + drz**2)
    error_list = [dx, dy, dz, drx, dry, drz]
    return error, error_list


def TwoPoints(t, P1, P2):
        
        if not isinstance(P1, np.ndarray) or not isinstance(P2, np.ndarray):
            raise TypeError('Points must be an instance of the numpy.ndarray!')
        if not isinstance(t, (int, float)):
            raise TypeError('Parameter t must be an int or float!')

        Q1 = (1 - t) * P1 + t * P2
        return Q1

def Points(t, points):
   
    newpoints = []
    for i1 in range(0, len(points) - 1):
        newpoints += [TwoPoints(t, points[i1], points[i1 + 1])]
    return newpoints

def Point(t, points):
 
    newpoints = points
    while len(newpoints) > 1:
        newpoints = Points(t, newpoints)
    return newpoints[0]

def publish():
    pub = rospy.Publisher('robotis/set_joint_states/', JointState, queue_size = 1)
    
    data = JointState()
    data.header.stamp = rospy.Time.now()
    data.name = ['l_hip_yaw', 'l_hip_roll', 'l_knee', 'l_ank_pitch_back', 'l_ank_roll']
    data.position = [-angle[0], angle[1], angle[2], angle[5], angle[6]]
    pub.publish(data)

def FK(angle):
    base = TF()
    hip_yaw_from_base = TF(rot_axis='z', q=angle[0], dy=0.105, dz=-0.02)
    hip_yaw = base.dot(hip_yaw_from_base) #R01
    hip_roll_from_hip_yaw = TF(rot_axis='x', q=angle[1], dx=0.05179, dz=-0.0425)
    hip_roll = hip_yaw.dot(hip_roll_from_hip_yaw) #R12
    hip_pitch_from_hip_roll = TF(rot_axis='y', q=angle[2])
    hip_pitch = hip_roll.dot(hip_pitch_from_hip_roll) #R23
    hip = hip_pitch # hip frame
    knee_from_hip = TF(rot_axis='y', q=-angle[2], dz=-0.275)
    knee_up = hip.dot(knee_from_hip) #R34
    kneeDown_from_up = TF(rot_axis='y', q=angle[5], dz=-0.16)
    knee_Down = knee_up.dot(kneeDown_from_up) 
    ankle_pitch_from_knee = TF(rot_axis='y', q=-angle[5], dz=-0.275)
    ankle_pitch = knee_Down.dot(ankle_pitch_from_knee) #R45
    ankle_roll_from_ankle_pitch = TF(rot_axis='x', q=-angle[1])
    ankle_roll = ankle_pitch.dot(ankle_roll_from_ankle_pitch) #R56
    ankle = ankle_roll
    sole_from_ankle = TF(dx=-0.051890, dz=-0.0455)
    sole = ankle.dot(sole_from_ankle) # sole frame R67
    f_result = kdl.Frame(kdl.Rotation(sole[0,0], sole[0,1], sole[0,2],
                                    sole[1,0], sole[1,1], sole[1,2],
                                    sole[2,0], sole[2,1], sole[2,2]),
                        kdl.Vector(sole[0,3], sole[1,3], sole[2,3]))
    xyz = [sole[0,3],sole[1,3],sole[2,3],angle[0]]

    return xyz, f_result, sole

def main(t_values, target, yawt):
    # Konfigurasi link
    global angle
    angle = []
    CROTCH_TO_HIP = 0.105    # Jarak croth ke hip
    UPPER_HIP = 0.0625       # Jarak hip yaw ke hip roll pitch
    HIP_TO_KNEE = 0.275      # Panjang link upper leg
    KNEE_TO_ANKLE = 0.275    # Panjang link lower leg
    ANKLE_TO_SOLE = 0.0455   # Jarak ankle ke sole
    UP_KNEE_TO_DOWN_KNEE = 0.16 #jarak joint pada lutut
    
    A = HIP_TO_KNEE
    B = KNEE_TO_ANKLE
    D = CROTCH_TO_HIP
    step = -1
    frame = np.eye(4) * len(target[0])
    Fsole = np.eye(4)
    figError = plt.figure()
    plt.grid()
    axError = figError.add_subplot()
    axError.set_xlabel("Step")
    axError.set_ylabel("Error")
    result = 0
    for t in t_values:
        sleep(0.02)
        
        xyz = target
        step = step+1
        # Input target
        x_from_base = xyz[step,0]
        y_from_base = xyz[step,1]
        z_from_base = xyz[step,2] 

        yaw = yawt[step,1]
        
        f_target = kdl.Frame(kdl.Rotation.RPY(0, 0, yaw), kdl.Vector(x_from_base, y_from_base, z_from_base))
        

        # Jarak target ke hip
        x_from_hip = (x_from_base - 0)
        y_from_hip = (y_from_base - CROTCH_TO_HIP)
        z_from_hip = (z_from_base + UPPER_HIP + ANKLE_TO_SOLE + UP_KNEE_TO_DOWN_KNEE)

        # jarak target ke hip dengan yaw
        xa = x_from_hip
        ya_1 = xa*np.tan(yaw)
        xb_1 = xa / np.cos(yaw)
        beta = np.radians(90) - yaw
        ya_2 = (y_from_hip-ya_1)
        yb = ya_2 * np.sin(beta)
        xb_2 = yb / np.tan(beta) #!90
        xb = xb_1 + xb_2

        x_from_hip = xb
        y_from_hip = yb
        z_from_hip = z_from_hip
        # Inverse Kinematic
        # Mencari jarak Target ke Hip (C)
        C = np.sqrt(x_from_hip**2+y_from_hip**2+z_from_hip**2)
        z_from_hip_roll = z_from_hip - 0.16
        qHipRoll = np.arctan2(y_from_hip, np.sign(z_from_hip_roll)*z_from_hip_roll)
        qAnkleRoll = -qHipRoll
        qroll = qHipRoll

        # Mencari sudut Knee
        q4 = -np.arccos(((A**2+B**2)-C**2)/(2*A*B))
        qrol2 = np.radians(180)-(np.radians(90)+qHipRoll) 
        qKnee = q4+np.radians(180)
        
        # Mencari sudut ankle pitch dan hip pitch
        q3 = qKnee/2
        qx = np.arcsin((x_from_hip)/C)
        qHipPitch = -(q3+qx)   
        
        q5 = q3
        qz = (np.sqrt(y_from_hip**2+z_from_hip**2))
        qAnklePitch = ((np.radians(180)-(np.arctan2(x_from_hip, np.sign(z_from_hip)*qz))) - q3)
        if qAnklePitch > np.radians(120) :
            qAnklePitch = qAnklePitch- np.radians(360)
        qHipYaw = yaw
        
        qKneeUp = qHipPitch 
        qKneeDown = qAnklePitch
        
        angle = [qHipYaw, qHipRoll, qHipPitch, qKneeUp, qKneeDown, -qAnklePitch,  qAnkleRoll]
        if np.isnan(qHipPitch):
            angle = [0,0,0,0,0,0,0]
        #print(angle)
        xyz2,f_result,sole = FK(angle)

        error,errorList = pose_error(f_target,f_result)
        
        print("C =", C)
        print("qroll",np.degrees(qroll))
        print("qroll2",np.degrees(qrol2))
        print("q3 = ",np.degrees(q3))
        print("qx =", np.degrees(qx))
        print("qz :", np.degrees(qz))
        
        print("Q hip Yaw 	: ", angle[0])
        print("Q hip Roll 	: ", angle[1])
        print("Q hip Pitch 	: ",angle[2])
        print("Q Knee UP	: ",angle[3])
        print("Q Knee Down 	: ",angle[4])
        print("Q ankle Pitch 	: ", angle[5])
        print("Q ankle Roll 	: ", angle[6])
        
        print("Input Position")
        print("x :", x_from_base)
        print("y :", y_from_base)
        print("z :", z_from_base)
        print("yaw :", np.degrees(yaw))
        
        print("Solution")
        print("x :", xyz2[0])
        print("y :", xyz2[1])
        print("z :", xyz2[2])
        print("yaw :", np.degrees(qHipYaw))
        
        print("ERROR :", error)
        print("=====================")
        print("ERROR LIST :", errorList)
        
        #print("(",x_from_base,y_from_base,z_from_base,") : (",xyz2[0],xyz2[1],xyz2[2],") : (",angle,":",error)
        
        if error < 0.01:
            result+=1
            status = "IK_Solved"
        else:
            status = "IK_Error"
         
        plt.scatter(step,error)
        Fsole = np.concatenate((Fsole, sole))
        rospy.init_node('marker_basic_node', anonymous=True)
        publish()
        
    soleList = Fsole
    N = 4
    Fsole = [soleList[n:n+N] for n in range(0, len(soleList), N)]
    return frame,Fsole
    

def Curve(t_values, points):
   
    if not hasattr(t_values, '__iter__'):
        raise TypeError("`t_values` Must be an iterable of integers or floats, of length greater than 0 .")
    if len(t_values) < 1:
        raise TypeError("`t_values` Must be an iterable of integers or floats, of length greater than 0 .")
    if not isinstance(t_values[0], (int, float)):
        raise TypeError("`t_values` Must be an iterable of integers or floats, of length greater than 0 .")

    curve = np.array([[0.0] * len(points[0])])
    for t in t_values:

        curve = np.append(curve, [Point(t, points)], axis=0)
    curve = np.delete(curve, 0, 0)
    return curve

points_set_1 = a([[-0.2, 0.105, -0.7], [-0.1, 0.105, -0.42], [0.1, 0.105, -0.42], [0.2, 0.105, -0.7]])
points_set_2 = a([[0, 0.105, -0.8], [0, 0.105, -0.69], [0, 0.305, -0.68], [0, 0.305, -0.68]])
points_set_3 = a([[0, 0.105, -0.8], [0, 0.105, -0.69], [0.142, 0.18, -0.68], [0.142, 0.18, -0.68]])

t_points = np.arange(0, 1, 0.01)

curve_set_1 = Curve(t_points, points_set_2) #set point trajectory

tyaw = np.radians(0) #yaw trajectory
time = 1

test = a([[0, 0], [0, tyaw], [time, 0], [time, tyaw]])
test_set_1 = Curve(t_points, test)

frame,sole = main(t_points, curve_set_1, test_set_1 ) #main program

figTrajectory = plt.figure(2)
axTrajectory = figTrajectory.add_subplot(projection='3d')
axTrajectory.set_xlim(-0.25,0.25)
axTrajectory.set_ylim(-0.2,0.3)
axTrajectory.set_zlim(-0.8,-0.3)
axTrajectory.set_xlabel("X axis")
axTrajectory.set_ylabel("Y axis")
axTrajectory.set_zlabel("Z axis")
cnt = 0
for t in t_points:
    cnt = cnt+1
    if cnt % 10 == 0 :
        draw_axis(axTrajectory,scale=0.05,A=sole[cnt])

plt.show()     
