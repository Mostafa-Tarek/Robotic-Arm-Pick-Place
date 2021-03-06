#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import sympy as sp
import numpy as np
from math import atan2, cos, sin, acos, pi
# Joint angle symbols
q0, q1, q2, q3, q4, q5, q6 = sp.symbols('q0:7') # # angle change between xi-1 axis and xi about zi
d0, d1, d2, d3, d4, d5, d6 = sp.symbols('d0:7') # distance between frames (x) along zi
a0, a1, a2, a3, a4, a5, a6 = sp.symbols('a0:7') # distance between frames (z) along xi-1
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = sp.symbols('alpha0:7') # angle change between zi-1 axis and zi about xi-1

# DH Parameters
'''
DH Table
Link    a       alpha   d       theta
1       a1      90      d1      *
2       a2      0       0       *
3       -a3     90      d3      *
4       0       -90     0       *
5       0       90      0       *
6       0       0       d6      *
'''
d_base = 0.75
a_1 = 0.35
a_2 = 1.25
a_3 = -0.054
d_3 = 1.5
d_6 = 0.303

# Create individual transformation matrices
def rot_x(r):
    '''
    Rotation about x axis
    :param angle: amount of rotation about x
    :return: rotation matrix
    '''
    R_x = sp.Matrix([[1, 0, 0, 0],
                     [0, sp.cos(r), -sp.sin(r), 0],
                     [0, sp.sin(r), sp.cos(r), 0],
                     [0, 0, 0, 1]])
    return R_x

def rot_y(p):
    '''
    Rotation about y axis
    :param angle: amount of rotation about y
    :return: rotation matrix
    '''
    R_y = sp.Matrix([[sp.cos(p), 0, sp.sin(p), 0],
                     [0, 1, 0, 0],
                     [-sp.sin(p), 0, sp.cos(p), 0],
                     [0, 0, 0, 1]])
    return R_y

def rot_z(y):
    '''
    Rotation about z axis
    :param angle: amount of rotation about z
    :return: rotation matrix
    '''
    R_z = sp.Matrix([[sp.cos(y), -sp.sin(y), 0, 0],
                     [sp.sin(y), sp.cos(y), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
    return R_z

def rpy_matrix(roll, pitch, yaw):
    '''
    The rotation matrix with Roll, Pitch and Yaw
    :param roll: angle about z axis
    :param pitch: angle about y axis
    :param yaw: angle about x axis
    :return: rotation matrix
    '''
    # roll : about z axis
    # pitch : about y axis
    # yaw : about x axis
    r11 = sp.cos(roll)*sp.cos(pitch)
    r12 = -sp.sin(roll)*sp.cos(yaw) + sp.cos(roll)*sp.sin(pitch)*sp.sin(yaw)
    r13 = sp.sin(roll)*sp.sin(yaw) + sp.cos(roll)*sp.sin(pitch)*sp.cos(yaw)
    r21 = sp.sin(roll)*sp.cos(pitch)
    r22 = sp.cos(roll)*sp.cos(yaw) + sp.sin(roll)*sp.sin(pitch)*sp.sin(yaw)
    r23 = -sp.cos(roll)*sp.sin(yaw) + sp.sin(roll)*sp.sin(pitch)*sp.cos(yaw)
    r31 = -sp.sin(pitch)
    r32 = sp.cos(pitch)*sp.sin(yaw)
    r33 = sp.cos(pitch)*sp.cos(yaw)

    return sp.Matrix([[r11, r12, r13, 0],
                      [r21, r22, r23, 0],
                      [r31, r32, r33, 0],
                      [0, 0, 0, 1]])

def limit_angle(theta, min, max):
    '''
    Check that angles are within rotation bounds
    :param theta: calculated angle
    :param min: lower bound threshold in degrees
    :param max: upper bound threshold in degrees
    :return: bounded angle
    '''
    if theta > pi * max / 180:
        return pi * max / 180
    elif theta < pi * min / 180:
        return pi * min / 180
    else:
        return theta
#Calculate the spherical wrist joint angles via Euler angles
def arm_IK(px, py, pz):
    '''
    Calculates the inverse kinematics of the arm section
    :param px: position along the x axis
    :param py: position along the y axis
    :param pz: position along the z axis
    :return: first three arm angles, theta 1, 2 and 3
    '''

    ''' theta 1 '''
    # Calculate joint angles using Geometric IK method
    theta1 = limit_angle(atan2(py, px), -185, 185)
    # calculate link 3 shoulder angle and distance to wrist center
    link_3 = np.sqrt(a_3 ** 2 + d_3 ** 2)
    link_3_theta = atan2(a_3, d_3)
    # remove base offsets from the wrist coords for theta 2 and 3
    a_1_x = a_1 * cos(theta1)  # link a_1 offset in x direction
    a_1_y = a_1 * sin(theta1)
    d_6_x = d_6 * cos(theta1)  # link d_6 offset in x direction
    d_6_y = d_6 * sin(theta1)
    # get the desired end arm x, y, z coordinates
    x_d = px - d_6_x - a_1_x
    y_d = py - a_1_y  # - d_6_y
    z_d = pz - d_base + d_6 / 3

    # xC wrist centre plane arm projections
    xC = np.sqrt(x_d ** 2 + y_d ** 2)
    # B length on 3D plane arm 
    B = np.sqrt(x_d ** 2 + y_d ** 2 + z_d ** 2)

    ''' theta 2 
        '''

    # link 1 to wrist center angle from z axis / vertical (pi/2)
    beta2 = atan2(xC, z_d)

    # law of cosine rule
    a_theta2 = (a_2 ** 2 + B ** 2 - link_3 ** 2) / (2 * a_2 * B)
    psi2 = atan2(np.sqrt(np.abs(1 - a_theta2 ** 2)), a_theta2)
    # zero angle is along z axis
    theta2 = limit_angle(beta2 - psi2, -45, 85)

    ''' theta 3 '''

    # law of cosine rule
    b_theta3 = (a_2 ** 2 + link_3 ** 2 - B ** 2) / (2 * a_2 * link_3)
    psi3 = atan2(np.sqrt(np.abs(1 - b_theta3 ** 2)), b_theta3) + link_3_theta
    # angle perpendicular wrt link 1 but psi is from link 1
    theta3 = limit_angle((psi3 - pi / 2) * -1.0, -210, 155 - 90)

    return theta1, theta2, theta3, psi2, beta2, psi3

def wrist_IK(px, py, pz, theta1, psi2, beta2, psi3):
    '''
    Calculates the inverse kinematics of the arm section
    :param px: position along the x axis
    :param py: position along the y axis
    :param pz: position along the z axis
    :param theta1: angle of the first rotation point
    :param psi2: angle 2 of the triangle of the arm section from the base
                   to the wrist
    :param beta2: angle between link1 and angle from base to arm wrist point
    :param psi3: angle 3 of the triangle of the arm section from the base
                   to the wrist
    :return: wrist angles, theta 4, 5 and 6
    '''

    ''' theta 4 '''
    # create a plane onto the shelf with origin in the center and inverted axes
    # rotate wrist around origin so axis 5 is the tangent of the circle from the origin

    z_o = (pz - (d_base + a_2)) * -1.  # origin of target shelf
    y_o = -py
    theta4 = -pi / 2 + atan2(z_o, y_o)
    theta4 = limit_angle(theta4, -350, 350)

    ''' theta 5 '''
    # keeps wrist level using geometric association laws
    psi5 = pi - psi2 - psi3
    beta5 = pi / 2 - beta2
    theta5 = -(psi5 - beta5)
    if z_o < 0:
        theta5 = -theta5
    if sin(theta1) > 0.55:
        theta5 -= pi / 2 * sin(theta1)

    theta5 = limit_angle(theta5, -125, 125)

    ''' theta 6 '''
    # rotate gripper keeping level wrt to the ground plane
    theta6 = -theta4
    theta6 = limit_angle(theta6, -350, 350)  # Extract end-effector position and orientation from request

    return theta4, theta5, theta6

def calculate_mod_FK(q0, q1, q2, q3, q4, q5):
	
    '''
    calculates the modified DH forward kinematics
    :param q0: angle of rotation of joint 1
    :param q1: angle of rotation of joint 2
    :param q2: angle of rotation of joint 3
    :param q3: angle of rotation of joint 4
    :param q4: angle of rotation of joint 5
    :param q5: angle of rotation of joint 6
    :return: Transformation matrix
    '''

    DH = {a0: 0,     alpha0: 0,          d0: d_base,
         a1: a_1,   alpha1: pi / 2,     d1: 0,      q1: q1 + pi/2,
         a2: a_2,   alpha2: 0,          d2: 0,
         a3: a_3,   alpha3: pi / 2,     d3: d_3,
         a4: 0,     alpha4: -pi / 2,    d4: 0,
         a5: 0,     alpha5: pi / 2,     d5: 0,
         a6: 0,     alpha6: 0,          d6: d_6,    q6: 0}
    # Modified DH params
    def Matrix_TF(a,alpha,d,q):
    	TF = sp.Matrix([[sp.cos(q), -sp.sin(q), 0, a],
                   [sp.sin(q) * sp.cos(alpha), sp.cos(q) * sp.cos(alpha), -sp.sin(alpha), -sp.sin(alpha) * d],
                   [sp.sin(q) * sp.sin(alpha), sp.cos(q) * sp.sin(alpha), sp.cos(alpha), sp.cos(alpha) * d],
                   [0, 0, 0, 1]])
    	TF = TF.subs(DH)
	return TF


    T0_1 = Matrix_TF(a0,alpha0,d0,q0)
    T1_2 = Matrix_TF(a1,alpha1,d1,q1)
    T2_3 = Matrix_TF(a2,alpha2,d2,q2)
    T3_4 = Matrix_TF(a3,alpha3,d3,q3)
    T4_5 = Matrix_TF(a4,alpha4,d4,q4)
    T5_6 = Matrix_TF(a5,alpha5,d5,q5)
    T6_7 = Matrix_TF(a5,alpha5,d5,q5)

    # Define Modified DH Transformation matrix
    T0_3 = sp.simplify(T0_1 * T1_2 * T2_3 * T3_4) # arm section
    T0_6 = sp.simplify(T0_3 * T4_5 * T5_6 * T6_7)  # Spherical wrist section

    return T0_6

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:


	# Extract rotation matrices from the transformation matrices

	#
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Arm
            theta1, theta2, theta3, alpha2, beta2, alpha3 = arm_IK(px, py, pz)
	    # Spherical wrist
            theta4, theta5, theta6 = wrist_IK(px, py, pz, theta1, alpha2, beta2, alpha3)  
            # Populate response for the IK request 
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)
	T0_6 = calculate_mod_FK(theta1, theta2, theta3, theta4, theta5, theta6)
        print("roll: ", roll, "   pitch: ", pitch, "   yaw: ", yaw)
        print("T0_6")
        # yaw, pitch, roll, R correction between frames
        # rpy_matrix(yaw, pitch, roll)
        Rrpy_cor = rot_y(pi / 2) * rot_z(pi)
        E = T0_6 * Rrpy_cor
        print(T0_6)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
