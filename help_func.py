#!/usr/bin/env python
from __future__ import division
import sys
import os
import get_ik
import rospy
import time
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
import random
from math import pi
import matplotlib.pyplot as plt
from shapely.geometry import *
from std_msgs.msg import *
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
import pybullet as p
import pybullet_data
import pybullet_utils
from collision_utils import get_collision_fn
import pybullet_planning as pp
import numpy as np
from scipy import interpolate
import scipy.io as sio
import IPython

UR10_JOINT_INDICES = [0, 1, 2, 3, 4, 5, 8] #movable joints of ur10_hong

def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)

def is_inPoly(polygen,points):
    # this is to determine is a point is in a polygen
    line = LineString(polygen)
    point = Point(points)
    polygen = Polygon(line)
    return polygen.contains(point)

def get_poly_from_gripper(center):
    #this is to get the gripper polygen of a chosen robot arm
    # center = group.get_current_pose().pose
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

def config_generate(group,center,robot_state,poly, num=20):
    # get collision free ik solutions from random sample, solutions stored in conf[]
    #step1: set up simulator
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))
    # load objects
    plane = p.loadURDF("plane.urdf")
    ur10_hong = p.loadURDF('assets/ur5/ur10_hong_gripper.urdf', basePosition=[0.885, 0.012, 0.786], baseOrientation=[0,0,0.70710678,0.70710678], useFixedBase=True)
    ur10_kong = p.loadURDF('assets/ur5/ur10_kong_gripper.urdf', basePosition=[-0.916, 0, 0.77], baseOrientation=[0,0,-0.70710678,0.70710678],useFixedBase=True)
    obstacle2 = p.loadURDF('assets/ur5/robot_movable_table.urdf',
                           basePosition=[0, 0, 0],
                           baseOrientation=[0,0,0,1],
                           useFixedBase=True)
    obstacle3 = p.loadURDF('assets/ur5/robot_optical_table_kong.urdf',
                           basePosition=[-1.381, 0, 0],
                           baseOrientation=[0,0,1,0],
                           useFixedBase=True)
    obstacles = [plane, ur10_kong, obstacle2, obstacle3]
    #step2: start and goal
    start_conf_kong = [-1.46, -2.36, -1.52, -0.82, 1.81, 2.65, 0.7]
    # start_conf_kong = [-1.75, -1.95, -1.34, -1.4, 1.7, 0.22, 0.7]
    set_joint_positions(ur10_kong, UR10_JOINT_INDICES, start_conf_kong)
    #step3: random sample configs
    ik = get_ik.GetIK(group)
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.z = 0.71 + 0.1411 #0.71(table height) + 0.11411 (dis_z between rogid_tip_link and hong_tool0)
    polygen = get_poly_from_gripper(center)
    a = np.arange(-0.12,0.12,0.0001)
    b = np.arange(-0.16,0.16,0.0001)
    c = np.arange(0,np.pi*2,np.pi/1000)
    conf = []
    i = 0
    while (i<num):
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
        m = ik.get_ik(pose,robot_state)
        if m == []:
            continue
        else:
            #step3: collision check
            set_joint_positions(ur10_hong, UR10_JOINT_INDICES, m[:7])
            collision_fn = get_collision_fn(ur10_hong, UR10_JOINT_INDICES, obstacles=obstacles,
                                            attachments=[], self_collisions=True,
                                            disabled_collisions=set())
            if collision_fn(m[:7]) == 0 and test_fk(m[:6],poly,robot_state) == 1:
                conf.append(m)
                i += 1
            else:
                continue
    conf = np.array(conf)
    conf = conf[:,:7]
    conf = conf.tolist()
    return conf

def test_fk(config,paper_polygen,robot_state):
    #test if rigid_tip_link1 is in the paper polygen
    ik_srv = rospy.ServiceProxy('/compute_fk', GetPositionFK)
    ik_srv.wait_for_service()

    header = Header()
    header.frame_id = "world"
    header.stamp = rospy.Time.now()
    fk_link_names = ["rigid_tip_link1"]
    # robot_state = robot.get_current_state()
    other_joint_values = [0.6974335690969342, -1.7504954265803478, -1.9503007193486717, -1.340203426021494,
                          -1.40115032350114, 1.6977166700000357, 0.21991148575129937, 0.6974335690969342]

    robot_state.joint_state.position = config + other_joint_values
    resp = ik_srv(header,fk_link_names,robot_state)
    pos = [resp.pose_stamped[0].pose.position.x,resp.pose_stamped[0].pose.position.y,
           resp.pose_stamped[0].pose.position.z]

    def is_inPoly(paper_polygen,pos):
        line = LineString(paper_polygen)
        point = Point(pos)
        polygen = Polygon(line)
        return polygen.contains(point)

    return is_inPoly(paper_polygen,pos)

def get_dis(configs,k_start_config,h_start_config,connect_mode=p.GUI):
    #input valid configs, obtain smallest distance for each config
    #step1: set up simulator
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))
    # load objects
    plane = p.loadURDF("plane.urdf")
    ur10_hong = p.loadURDF('assets/ur5/ur10_hong_gripper.urdf', basePosition=[0.885, 0.012, 0.786], baseOrientation=[0,0,0.70710678,0.70710678], useFixedBase=True)
    ur10_kong = p.loadURDF('assets/ur5/ur10_kong_gripper.urdf', basePosition=[-0.916, 0, 0.77], baseOrientation=[0,0,-0.70710678,0.70710678],useFixedBase=True)
    obstacle2 = p.loadURDF('assets/ur5/robot_movable_table.urdf',
                           basePosition=[0, 0, 0],
                           baseOrientation=[0,0,0,1],
                           useFixedBase=True)
    obstacle3 = p.loadURDF('assets/ur5/robot_optical_table_kong.urdf',
                           basePosition=[-1.381, 0, 0],
                           baseOrientation=[0,0,1,0],
                           useFixedBase=True)
    obstacles = [plane, ur10_kong, obstacle2, obstacle3]
    #step2: set start configs
    start_conf_kong = k_start_config
    start_conf_hong = h_start_config
    set_joint_positions(ur10_hong, UR10_JOINT_INDICES, start_conf_hong)
    set_joint_positions(ur10_kong, UR10_JOINT_INDICES, start_conf_kong)
    #step3: get min dis list
    min_dis = []
    for c in configs:
        goal_conf_hong = c
        path, dis = pp.plan_joint_motion(ur10_hong,UR10_JOINT_INDICES,goal_conf_hong,obstacles)
        if dis is None:
            min_dis.append(0)
            continue
        min_dis_tmp = min(dis)
        min_dis.append(min_dis_tmp)
    return min_dis
