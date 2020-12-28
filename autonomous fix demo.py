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
import pybullet as p
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor, Constraints, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from std_msgs.msg import String,Empty,UInt16
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from std_msgs.msg import *
import random
from math import pi
import matplotlib.pyplot as plt
from shapely.geometry import *
import help_func as hf
import test_config as tc
import test_planner as tp

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('dual_arm_origami', anonymous=True)
listener = tf.TransformListener()
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(1)
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

def robhong_go_to_home(state):
    if state == 1:
        joint_values = [-66.86/180*pi, -69.6/180*pi, -120.2/180*pi, -51.78/180*pi, 77.87/180*pi, 111.00/180*pi]
    elif state == 2:
        joint_values = []
    group1.set_joint_value_target(joint_values)
    plan = group1.plan()
    print "============ hong_home"
    rospy.sleep(2)
    scaled_traj2 = scale_trajectory_speed(plan, 0.3)
    group1.execute(scaled_traj2)

def robkong_go_to_home(state):
    if state == 1:
        joint_values = [45.80/180*pi, -108.10/180*pi, 111.35/180*pi, -113.3/180*pi, -72.05/180*pi, 47.2/180*pi]
    elif state == 2:
        joint_values = []
    group2.set_joint_value_target(joint_values)
    plan = group2.plan()
    print "============ kong_home"
    rospy.sleep(2)
    scaled_traj2 = scale_trajectory_speed(plan, 0.3)
    group2.execute(scaled_traj2)

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

#####################initialization#########################
addCollisionObjects()
rospy.sleep(1)
robkong_go_to_home(1)
robhong_go_to_home(1)
hybridPub(0.6981,"robhong")
hybridPub(0.6981,"robkong")
# scene.remove_world_object(paper_name)
#####################initialization#########################

#####################grasp position##########################
# grasp = np.array([42.37,-70.35,77.17,-104.42,-66.41,39.65])
grasp = []
grasp = grasp / 180 * np.pi
group2.set_joint_value_target(grasp)
plan = group2.plan()
new_traj = scale_trajectory_speed(plan, 0.2)
group2.execute(new_traj)
group2.stop()
group2.clear_pose_targets()
#####################grasp position##########################

#####################generate fix configuration#######################
# step1: get apriltag transformation
(trans_tag,rot_tag) = listener.lookupTransform("world", "tag_15", rospy.Time(0))
tag_pos = [trans_tag[0],trans_tag[1],0.71]
tag_rot_mat = tf.transformations.quaternion_matrix(rot_tag)
tag_rot = tag_rot_mat[:2,:2]
rospy.sleep(1)
# step2: input hong's and kong's configurations in grasp process
k1 = np.array([58.8,-42.88,82.66,-128.92,-86.37,98.64,40.11]) #k1 and k2 are example configs in a flex process
k1 = k1 / 180 * np.pi
k2 = np.array([59.83,-50.17,96.95,-136.00,-86.27,99.68,40.11])
k2 = k2 / 180 * np.pi
k_start = [k1,k2]
h = np.array([-79.27,-77.13,-123.84,-56.66,97.35,-46.48,40.11]) #h is an example start config of hong
h = h / 180 *np.pi
h_start = h
# step3: kong goes to grasp configuration
group2.set_joint_value_target(k1[:6])
plan = group2.plan()
new_traj = scale_trajectory_speed(plan, 0.2)
group2.execute(new_traj)
group2.stop()
group2.clear_pose_targets()
# step4: get information needed for config_generate
center = group2.get_current_pose().pose
robot_state = robot.get_current_state()
configs = hf.config_generate(group_name1,center,k_start,robot_state,tag_pos,tag_rot,num=20,sim=True) #get valid configurations
p.disconnect()
dis = hf.get_dis(configs,k_start,h_start,p.DIRECT) #choose the optimal config
p.disconnect()
sid = sorted(range(len(dis)),key=lambda k:dis[k], reverse=True)
goal_conf = configs[sid[0]][:6]
# print "-------%s--------", time.time() - start_time
print "goal_conf",goal_conf
#####################generate fix configuration#######################

#####################hong goes to goal configuration###############################
group1.set_joint_value_target(goal_conf[:6])
plan = group1.plan()
new_traj = scale_trajectory_speed(plan, 0.2)
group1.execute(new_traj)
group1.stop()
group1.clear_pose_targets()
#####################hong goes to goal configuration###############################
