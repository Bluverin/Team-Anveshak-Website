#!/usr/bin/env python3

import rospy
import numpy as np
import PyKDL as kdl

import  kdl_parser_py.urdf as kdl_parser
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from math import *

#global variables
(status, tree) = kdl_parser.treeFromFile("/home/bluverin/ik/src/ik_pkg/config/two_link_arm.urdf")
chain = tree.getChain("base_link", "tip_1")
num_joints = chain.getNrOfJoints()
fk_pos_solver = kdl.ChainFkSolverPos_recursive(chain)
ee_pose = kdl.Frame()
q   = np.zeros(num_joints)

def joint_list_to_kdl(q):
     if q is None:
         return None
     if type(q) == np.matrix and q.shape[1] == 0:
        q = q.T.tolist()[0]
     q_kdl = kdl.JntArray(len(q))
     for i, q_i in enumerate(q):
         q_kdl[i] = q_i
     return q_kdl

def endeff_pose(data):
        global ee_pose, joint_values
        joint_values = data.position
        fk_pos_solver.JntToCart(joint_list_to_kdl(joint_values), ee_pose)  
        print(ee_pose.p)


if __name__ == "__main__":
    rospy.init_node("iksolver_node", anonymous=True)
    rospy.Subscriber("/joint_states", JointState, endeff_pose)
    rospy.spin()

