#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
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
from mpmath import *
from sympy import *


# alpha_i-1, a_i-1, d_i, q_i
def Tmatrix(alpha, a, d, q):
    """
    return
        Matrix([[            cos(q),           -sin(q),           0,             a],
                [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
    """

    m1 = Matrix([[     1,          0,           0,  a],
                 [     0, cos(alpha), -sin(alpha),  0],
                 [     0, sin(alpha),  cos(alpha),  0],
                 [     0,          0,           0,  1]])

    m2 = Matrix([[cos(q),    -sin(q),           0,  0],
                 [sin(q),     cos(q),           0,  0],
                 [     0,          0,           1,  d],
                 [     0,          0,           0,  1]])
    return m1*m2


def Rx(roll):
    return Matrix([[          1,          0,          0],
                   [          0,  cos(roll), -sin(roll)],
                   [          0,  sin(roll),  cos(roll)]])

def Ry(pitch):
    return Matrix([[ cos(pitch),          0, sin(pitch)],
                   [          0,          1,          0],
                   [-sin(pitch),          0, cos(pitch)]])

def Rz(yaw):
    return Matrix([[    cos(yaw), -sin(yaw),          0],
                   [    sin(yaw),  cos(yaw),          0],
                   [           0,         0,          1]])

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

        chosen_sinq3 = 0
        chosen_cosq3 = 0
        chosen_sinq2 = 0
        chosen_cosq2 = 0

        # Define DH param symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


        # Joint angle symbols
        s = {alpha0:     0,  a0:      0, d1:  0.75,
             alpha1: -pi/2,  a1:   0.35, d2:     0,  q2: q2-pi/2,
             alpha2:     0,  a2:   1.25, d3:     0,
             alpha3: -pi/2,  a3: -0.054, d4:  1.50,
             alpha4:  pi/2,  a4:      0, d5:     0,
             alpha5: -pi/2,  a5:      0, d6:     0,
             alpha6:     0,  a6:      0, d7: 0.303,  q7: 0}

        #print "s",s

        # Modified DH params
        T0_1 = Tmatrix(alpha0, a0, d1, q1)
        T0_1 = T0_1.subs(s)

        T1_2 = Tmatrix(alpha1, a1, d2, q2)
        T1_2 = T1_2.subs(s)

        T2_3 = Tmatrix(alpha2, a2, d3, q3)
        T2_3 = T2_3.subs(s)

        """
        T3_4 = Tmatrix(alpha3, a3, d4, q4)
        T3_4 = T3_4.subs(s)

        T4_5 = Tmatrix(alpha4, a4, d5, q5)
        T4_5 = T4_5.subs(s)

        T5_6 = Tmatrix(alpha5, a5, d6, q6)
        T5_6 = T5_6.subs(s)

        T6_7 = Tmatrix(alpha6, a6, d7, q7)
        T6_7 = T6_7.subs(s)
        """

        # Define Modified DH Transformation matrix
        T0_2 = T0_1 * T1_2
        T0_3 = T0_2 * T2_3
        """
        T0_4 = T0_3 * T3_4

        T0_5 = T0_4 * T4_5
        T0_6 = T0_5 * T5_6
        T0_7 = T0_6 * T6_7
        """

        # Create individual transformation matrices
        """
        T_z = Matrix([[    cos(pi), -sin(pi),          0, 0],
                      [    sin(pi),  cos(pi),          0, 0],
                      [          0,        0,          1, 0],
                      [          0,        0,          0, 1]])

        T_y = Matrix([[ cos(-pi/2),        0, sin(-pi/2), 0],
                      [          0,        1,          0, 0],
                      [-sin(-pi/2),        0, cos(-pi/2), 0],
                      [          0,        0,          0, 1]])

        T_corr = T_z * T_y

        T_total = T0_7 * T_corr
        """

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            print "x",x
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method

            #Te = tf.transformations.euler_matrix(roll, pitch, yaw)
            Tq = tf.transformations.quaternion_matrix(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Extrinsic rotation, reverse order
            #Re = Rz(yaw)*Ry(pitch)*Rx(roll)
            #print "rpyRe",Re
            #print "euler_matrix", Te[:,0:3][0:3,:]
            #print "quaternion_matrix", Tq[:,0:3][0:3,:]
            Re = Tq[:,0:3][0:3,:] # Use the matrix from quaternion
            I = Re[:,0]

            wx = px - (d7)*I[0]
            wy = py - (d7)*I[1]
            wz = pz - (d7)*I[2]

            wx = wx.subs(s)
            wy = wy.subs(s)
            wz = wz.subs(s)

            theta1 = N(atan2(wy, wx))

            # To obtain theta3, use the equations obtained from translation
            # component in T0_4, making them equal to (wx, wy, wz).
            # Combining these equations, we obtained:
            #   (wx/cos(q1) - a1)**2 + (wz - d1)**2 =
            #         d4**2 + a3**2 + a2**2 - 2*a2*a3*cos(q3) - 2*a2*d4*sin(q3)
            # Then solve using u = tan(q3/2)
            # A quadratic equation in u is obtained with 2 solutions for u,
            # hence 2 versions of sin(q3) (and cos(q3)) are obtained
            k = (d4**2 + a3**2 + a2**2 - (wx/cos(theta1)-a1)**2 - (wz-d1)**2)/(2*a2)
            k = k.subs(s)

            # First versions of sin(q3) and cos(q3)
            """
            u = (2*d4 + sqrt(4*d4**2 - 4*(k**2 - a3**2)))/(2*(k + a3))
            u = u.subs(s)

            sinq3_v1 = 2*u/(1 + u**2)
            cosq3_v1 = (1 - u**2)/(1 + u**2)
            """

            # Second versions of sin(q3) and cos(q3)
            # This version will get the more sensible arm pose
            # for the given configuration
            u = (2*d4 - sqrt(4*d4**2 - 4*(k**2 - a3**2)))/(2*(k + a3))
            u = u.subs(s)

            sinq3_v2 = 2*u/(1 + u**2)
            cosq3_v2 = (1 - u**2)/(1 + u**2)

            # For theta2, from the equations aforementioned, we obtain:
            #   (wx*sin(q2)/cos(q1) + wz*cos(q2) =
            #         a2 - a3*cos(q3) - d4*sin(q3) + a1*sin(q2) + d1*cos(q2)
            # Then we solve using the same procedure to obtain q3

            # First versions of sin(q2) and cos(q2)
            # This version will get the more sensible arm pose
            # for the given configuration
            u = (-2*(wx/cos(theta1) - a1) + sqrt(4*(wx/cos(theta1) - a1)**2 - 4*(d1 - wz - a2 + k)*(wz - d1 - a2 + k)))/(2*(d1 - wz - a2 + k))
            u = u.subs(s)

            sinq2_v1 = 2*u/(1 + u**2)
            cosq2_v1 = (1 - u**2)/(1 + u**2)

            # Second versions of sin(q2) and cos(q2)
            """
            u = (-2*(wx/cos(theta1) - a1) - sqrt(4*(wx/cos(theta1) - a1)**2 - 4*(d1 - wz - a2 + k)*(wz - d1 - a2 + k)))/(2*(d1 - wz - a2 + k))
            u = u.subs(s)

            sinq2_v2 = 2*u/(1 + u**2)
            cosq2_v2 = (1 - u**2)/(1 + u**2)
            """

            chosen_sinq3 = sinq3_v2
            chosen_cosq3 = cosq3_v2
            chosen_sinq2 = sinq2_v1
            chosen_cosq2 = cosq2_v1

            """
            chosen_sinq3 = sinq3_v1
            chosen_cosq3 = cosq3_v1
            chosen_sinq2 = sinq2_v2
            chosen_cosq2 = cosq2_v2
            print "choosing q3_v1 q2_v2"
            """
            
            """
            print "c6"
            # Now we check which combination of solutions are valid
            tolerance = 0.0000001
            #print "--------- q3_v1 q2_v1 ----------------"
            _wz = -d4*(sinq2_v1*cosq3_v1 + cosq2_v1*sinq3_v1) + a2*cosq2_v1 - a3*(cosq2_v1*cosq3_v1 - sinq2_v1*sinq3_v1) + d1
            _wz = _wz.subs(s)
            #print "wz",wz,"_wz",_wz

            _wx = cos(theta1)*(a2*sinq2_v1 - a3*(sinq2_v1*cosq3_v1 + cosq2_v1*sinq3_v1) + d4*(cosq2_v1*cosq3_v1 - sinq2_v1*sinq3_v1) + a1)
            _wx = _wx.subs(s)
            #print "wx",wx,"_wx",_wx

            _wy = sin(theta1)*(a2*sinq2_v1 - a3*(sinq2_v1*cosq3_v1 + cosq2_v1*sinq3_v1) + d4*(cosq2_v1*cosq3_v1 - sinq2_v1*sinq3_v1) + a1)
            _wy = _wy.subs(s)
            #print "wy",wy,"_wy",_wy

            min_dist = 10000

            # We choose the solutions whose positions (obtained after
            # evaluating the translation component of T0_4) have a distance
            # to the wrist position smaller than a tolerance value
            if abs(_wz - wz) < tolerance and abs(_wx - wx) < tolerance and abs(_wy - wy) < tolerance:
                # Additionally, we choose the solutions that are closest to the
                # sin and cos obtained in the previous iteration
                dist = (sinq3_v1 - chosen_sinq3)**2 + (cosq3_v1 - chosen_cosq3)**2 + (sinq2_v1 - chosen_sinq2)**2 + (cosq2_v1 - chosen_cosq2)**2
                if dist < min_dist:
                    min_dist = dist
                    chosen_sinq3 = sinq3_v1
                    chosen_cosq3 = cosq3_v1
                    chosen_sinq2 = sinq2_v1
                    chosen_cosq2 = cosq2_v1
                    print "choosing q3_v1 q2_v1, min_dist", min_dist

            #print "--------- q3_v2 q2_v1 ----------------"
            _wz = -d4*(sinq2_v1*cosq3_v2 + cosq2_v1*sinq3_v2) + a2*cosq2_v1 - a3*(cosq2_v1*cosq3_v2 - sinq2_v1*sinq3_v2) + d1
            _wz = _wz.subs(s)
            #print "wz",wz,"_wz",_wz

            _wx = cos(theta1)*(a2*sinq2_v1 - a3*(sinq2_v1*cosq3_v2 + cosq2_v1*sinq3_v2) + d4*(cosq2_v1*cosq3_v2 - sinq2_v1*sinq3_v2) + a1)
            _wx = _wx.subs(s)
            #print "wx",wx,"_wx",_wx

            _wy = sin(theta1)*(a2*sinq2_v1 - a3*(sinq2_v1*cosq3_v2 + cosq2_v1*sinq3_v2) + d4*(cosq2_v1*cosq3_v2 - sinq2_v1*sinq3_v2) + a1)
            _wy = _wy.subs(s)
            #print "wy",wy,"_wy",_wy

            if abs(_wz - wz) < tolerance and abs(_wx - wx) < tolerance and abs(_wy - wy) < tolerance:
                dist = (sinq3_v2 - chosen_sinq3)**2 + (cosq3_v2 - chosen_cosq3)**2 + (sinq2_v1 - chosen_sinq2)**2 + (cosq2_v1 - chosen_cosq2)**2
                if dist < min_dist:
                    min_dist = dist
                    chosen_sinq3 = sinq3_v2
                    chosen_cosq3 = cosq3_v2
                    chosen_sinq2 = sinq2_v1
                    chosen_cosq2 = cosq2_v1
                    print "choosing q3_v2 q2_v1, min_dist", min_dist

            #print "--------- q3_v1 q2_v2 ----------------"
            _wz = -d4*(sinq2_v2*cosq3_v1 + cosq2_v2*sinq3_v1) + a2*cosq2_v2 - a3*(cosq2_v2*cosq3_v1 - sinq2_v2*sinq3_v1) + d1
            _wz = _wz.subs(s)
            #print "wz",wz,"_wz",_wz

            _wx = cos(theta1)*(a2*sinq2_v2 - a3*(sinq2_v2*cosq3_v1 + cosq2_v2*sinq3_v1) + d4*(cosq2_v2*cosq3_v1 - sinq2_v2*sinq3_v1) + a1)
            _wx = _wx.subs(s)
            #print "wx",wx,"_wx",_wx

            _wy = sin(theta1)*(a2*sinq2_v2 - a3*(sinq2_v2*cosq3_v1 + cosq2_v2*sinq3_v1) + d4*(cosq2_v2*cosq3_v1 - sinq2_v2*sinq3_v1) + a1)
            _wy = _wy.subs(s)
            #print "wy",wy,"_wy",_wy

            if abs(_wz - wz) < tolerance and abs(_wx - wx) < tolerance and abs(_wy - wy) < tolerance:
                dist = (sinq3_v1 - chosen_sinq3)**2 + (cosq3_v1 - chosen_cosq3)**2 + (sinq2_v2 - chosen_sinq2)**2 + (cosq2_v2 - chosen_cosq2)**2
                if dist < min_dist:
                    min_dist = dist
                    chosen_sinq3 = sinq3_v1
                    chosen_cosq3 = cosq3_v1
                    chosen_sinq2 = sinq2_v2
                    chosen_cosq2 = cosq2_v2
                    print "choosing q3_v1 q2_v2, min_dist", min_dist

            #print "--------- q3_v2 q2_v2 ----------------"
            _wz = -d4*(sinq2_v2*cosq3_v2 + cosq2_v2*sinq3_v2) + a2*cosq2_v2 - a3*(cosq2_v2*cosq3_v2 - sinq2_v2*sinq3_v2) + d1
            _wz = _wz.subs(s)
            #print "wz",wz,"_wz",_wz

            _wx = cos(theta1)*(a2*sinq2_v2 - a3*(sinq2_v2*cosq3_v2 + cosq2_v2*sinq3_v2) + d4*(cosq2_v2*cosq3_v2 - sinq2_v2*sinq3_v2) + a1)
            _wx = _wx.subs(s)
            #print "wx",wx,"_wx",_wx

            _wy = sin(theta1)*(a2*sinq2_v2 - a3*(sinq2_v2*cosq3_v2 + cosq2_v2*sinq3_v2) + d4*(cosq2_v2*cosq3_v2 - sinq2_v2*sinq3_v2) + a1)
            _wy = _wy.subs(s)
            #print "wy",wy,"_wy",_wy

            if abs(_wz - wz) < tolerance and abs(_wx - wx) < tolerance and abs(_wy - wy) < tolerance:
                dist = (sinq3_v2 - chosen_sinq3)**2 + (cosq3_v2 - chosen_cosq3)**2 + (sinq2_v2 - chosen_sinq2)**2 + (cosq2_v2 - chosen_cosq2)**2
                if dist < min_dist:
                    min_dist = dist
                    chosen_sinq3 = sinq3_v2
                    chosen_cosq3 = cosq3_v2
                    chosen_sinq2 = sinq2_v2
                    chosen_cosq2 = cosq2_v2
                    print "choosing q3_v2 q2_v2, min_dist", min_dist
            """

            theta3 = atan2(chosen_sinq3, chosen_cosq3)
            theta2 = atan2(chosen_sinq2, chosen_cosq2)

            print "theta1",theta1,"theta2",theta2,"theta3",theta3

            T0_3_no_sym = T0_3.subs({q1: theta1, q2: theta2, q3: theta3})
            R0_3 = T0_3_no_sym[:,0:3][0:3,:]
            R3_7 = R0_3.T * Re

            theta4 = N(atan2(R3_7[2,0], -R3_7[0,0]))
            theta5 = N(atan2(sqrt(R3_7[2,0]**2 + R3_7[0,0]**2), R3_7[1,0]))
            theta6 = N(atan2(R3_7[1,1], R3_7[1,2]))
            print "theta4",theta4,"theta5",theta5,"theta6",theta6

            # FINAL PROOF: T_total rotation matrix has to be equal to Re,
            # and T_total translation component has to be equal to (px,py,pz)
            #T_total_no_sym = T_total.subs({q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
            #print "T_total.subs",T_total_no_sym
            #print "RE", Re
            #print "px",px,"py",py,"pz",pz
            print "******************************************"

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    rospy.loginfo("READY TO RECEIVE IK REQUEST")
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
