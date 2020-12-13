#!/usr/bin/env python

import sys
import get_ik
import copy
import moveit_commander
import rospy
import moveit_msgs.msg
import time
import tf
import numpy as np
import geometry_msgs.msg
import trajectory_msgs.msg
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor, Constraints, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
from std_msgs.msg import String,Empty,UInt16
import random
from math import pi
import matplotlib.pyplot as plt
from shapely.geometry import *

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('dual_arm_origami', anonymous=True)
listener = tf.TransformListener()
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2)
group_name1 = "hong_arm"
group1 = moveit_commander.MoveGroupCommander(group_name1)
group_name2 = "kong_arm"
group2 = moveit_commander.MoveGroupCommander(group_name2)
group_name3 = "hong_hand"
group3 = moveit_commander.MoveGroupCommander(group_name3)
group_name4 = "kong_hand"
group4 = moveit_commander.MoveGroupCommander(group_name4)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

def scale_trajectory_speed(traj, scale):
    new_traj = moveit_msgs.msg.RobotTrajectory()
    new_traj.joint_trajectory = traj.joint_trajectory
    n_joints = len(traj.joint_trajectory.joint_names)
    n_points = len(traj.joint_trajectory.points)
    points = list(traj.joint_trajectory.points)

    for i in range(n_points):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale
        points[i] = point

    new_traj.joint_trajectory.points = points
    return new_traj

def robkong_go_to_home(times,vel):
    if times == 1:
        joint_values = [-1.75, -1.95, -1.34, -1.4, 1.7, 0.22]
    elif times == 2:
        joint_values = [-2.03, -1.1, 1.93, -2.42, -1.5, 1.23]
    group2.set_joint_value_target(joint_values)
    plan = group2.plan()
    new_traj = scale_trajectory_speed(plan, vel)
    group2.execute(new_traj)
    group2.stop()
    group2.clear_pose_targets()

def robhong_go_to_home(times,vel):
    if times == 1:
        joint_values = [0.96, -1.07, 1.37, -1.31, -1.7, 0.32]
    elif times == 2:
        joint_values = [-2.04,-1.28,2.33,-2.69,-1.56,0.84]
    group1.set_joint_value_target(joint_values)
    plan = group1.plan()
    new_traj = scale_trajectory_speed(plan, vel)
    group1.execute(new_traj)
    group1.stop()
    group1.clear_pose_targets()

def hybridPub(joint_value,robot_arm):
    if robot_arm == "robhong":
        group = group3
    elif robot_arm == "robkong":
        group = group4
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0
    group.set_joint_value_target([joint_value])
    group.go(wait=True)

def addCollisionObjects():
    global paper_pose
    global paper_name
    paper_pose = geometry_msgs.msg.PoseStamped()
    paper_pose.header.frame_id = "world"
    paper_pose.pose.orientation.w = 1.0
    paper_pose.pose.position.x = 0
    paper_pose.pose.position.y = 0
    paper_pose.pose.position.z = 0.706
    paper_name = 'paper'
    scene.add_box(paper_name, paper_pose, (0.21, 0.297, 0.002))

def is_inPoly(polygen,points):
    # this is to determine is a point is in a polygen
    line = LineString(polygen)
    point = Point(points)
    polygen = Polygon(line)
    return polygen.contains(point)

def get_poly_from_gripper(robot_arm):
    #this is to get the gripper polygen of a chosen robot arm
    if robot_arm == "robhong":
        group = group1
    elif robot_arm == "robkong":
        group = group2
    else:
        print "Input error, please input a valid robot arm!"
        return 0
    center = group.get_current_pose().pose
    c_pos = np.array([center.position.x,center.position.y,center.position.z])
    c_ori = [center.orientation.x,center.orientation.y,center.orientation.z,center.orientation.w]
    c_mat = tf.transformations.quaternion_matrix(c_ori)
    c_mat = c_mat.transpose()
    x_axis = c_mat[0]
    x_axis = x_axis[:3]
    x_axis[2] = 0
    y_axis = c_mat[1]
    y_axis = y_axis[:3]
    y_axis[2] = 0
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    p1 = c_pos + 0.04*x_axis + 0.08*y_axis
    p2 = c_pos + 0.04*x_axis - 0.08*y_axis
    p3 = c_pos - 0.04*x_axis - 0.08*y_axis
    p4 = c_pos - 0.04*x_axis + 0.08*y_axis
    return p1, p2, p3, p4

############_____INITIALIZATION_____############
addCollisionObjects()
rospy.sleep(1)
robkong_go_to_home(1,0.4)
robhong_go_to_home(1,0.4)
hybridPub(0.6981,"robhong")
hybridPub(0.6981,"robkong")
# scene.remove_world_object(paper_name)
############_____INITIALIZATION_____############

# get collision free ik solutions from random sample, solutions stored in conf[]
ik = get_ik.GetIK(group_name1)
pose = PoseStamped()
pose.header.frame_id = "world"
pose.header.stamp = rospy.Time.now()
pose.pose.position.z = 0.71 + 0.1411 #0.71(table height) + 0.11411 (dis_z between rogid_tip_link and hong_tool0)
polygen = get_poly_from_gripper("robkong")
start_time = time.time()
a = np.arange(-0.105,0.105,0.0001)
b = np.arange(-0.145,0.145,0.0001)
c = np.arange(0,np.pi*2,np.pi/1000)
conf = []
i = 0
while (i<40):
    x = random.choice(a)
    y = random.choice(b)
    pose.pose.position.x = x
    pose.pose.position.y = y
    if is_inPoly(polygen,(x,y,0.87)) == 1:
        continue
    theta = random.choice(c)
    mat = [[np.cos(theta),np.sin(theta),0,0],[np.sin(theta),-np.cos(theta),0,0],[0,0,-1,0],[0,0,0,1]]
    ori = tf.transformations.quaternion_from_matrix(mat)
    pose.pose.orientation.x = ori[0]
    pose.pose.orientation.y = ori[1]
    pose.pose.orientation.z = ori[2]
    pose.pose.orientation.w = ori[3]
    m = ik.get_ik(pose,robot)
    if m == []:
        continue
    conf.append(m)
    i += 1
print "---%s seconds ----", time.time()-start_time

#test
print "-------------test--------------"
print "please see the arm's movement in the Moveit"
joint_values = conf[7][:6]
print "the joint values of arm hong is ", joint_values
group1.set_joint_value_target(joint_values)
group1.go()
