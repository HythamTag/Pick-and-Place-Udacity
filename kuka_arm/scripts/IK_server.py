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
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8')
    d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
    a0,a1,a2,a3,a4,a5,a6 = symbols('a0:6')
    alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')
	# Create Modified DH parameters
    s = { alpha0:      0,      a0:     0,      d1: 0.75
          alpha0:  -pi/2,      a0:  0.35,      d1: 0
          alpha0:      0,      a0:  1.25,      d1: 0
          alpha0:  -pi/2,      a0:-0.054,      d1: 1.5
          alpha0:   pi/2,      a0:     0,      d1: 0
          alpha0:  -pi/2,      a0:     0,      d1: 0
          alpha0:      0,      a0:     0,      d1: 0.303 }
	# Define Modified DH Transformation matrix
	def Transform(q,d,a,alpha,s):
        T = Matrix([[cos(q)             , -sin(q)            ,  0         , a              ],
                    [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                    [sin(q) * sin(alpha), cos(q) * sin(alpha),  cos(alpha),  cos(alpha) * d ],
                    [0                  , 0                  ,  0         ,  1              ]])

        return T.subs(s)
	# Create individual transformation matrices
	T0_1=Transform(q1,d1,a0,alpha0)
	T0_2=Transform(q2,d2,a1,alpha1)
	T0_3=Transform(q3,d3,a2,alpha2)
	T0_4=Transform(q4,d4,a3,alpha3)
	T0_5=Transform(q5,d5,a4,alpha4)
	T0_6=Transform(q6,d6,a5,alpha5)
	T0_G=Transform(q7,d7,a6,alpha6)
	# Extract rotation matrices from the transformation matrices
	#
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
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

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
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
