
import numpy as np
from DH_homog import dh_homog
import math
import constant


#  ###############################################################################################################
# calculates the homogenous transformation matrix and Jacobian of the arm
# input: list of the joint variables [arm_displacement (mm), head_displacement (mm), pan (rad), tilt (rad)]
# output: Numpy matrices of the Transformation Matrix and the associated Jacobian (unused in non trajectory design)
#  ###############################################################################################################
def arm_fwdk(joint_vars):

    # calculate the frame transformation matrices
    trans_0to1 = dh_homog(0., constant.ARM_HEIGHT_DISPLACEMENT + joint_vars[0], 0., -math.pi / 2)
    trans_1to2 = dh_homog(0., constant.SCISSOR_DISPLACEMENT + joint_vars[1],    0., math.pi / 2)
    trans_2to3 = dh_homog(joint_vars[2] + math.pi / 2,          0.,             0., math.pi / 2)
    trans_3to4 = dh_homog(joint_vars[3] - math.pi/2,            0.,             0., -math.pi / 2)

    # total Transformation Matrix
    trans_0to4 = trans_0to1 @ trans_1to2 @ trans_2to3 @ trans_3to4

    # DH convention k vectors
    k0 = np.array([[0.], [0.], [1.], [0.]])
    k1 = trans_0to1 @ k0
    k2 = trans_0to1 @ trans_1to2 @ k0
    k3 = trans_0to1 @ trans_1to2 @ trans_2to3 @ k0

    # origin locations of each joint
    o0 = np.array([[0.], [0.], [0.], [1.]])
    o1 = trans_0to1 @ o0
    o2 = trans_0to1 @ trans_1to2 @ o0
    o3 = trans_0to1 @ trans_1to2 @ trans_2to3 @ o0
    o4 = trans_0to4 @ o0

    # calculate columns of the Jacobian matrix
    # ji = [ [ki x (od - oi)] ; [ki] ]
    j1 = np.block([[np.cross(k0[0:3].T, (o4[0:3] - o0[0:3]).T).T], [k0[0:3]]])
    j2 = np.block([[np.cross(k1[0:3].T, (o4[0:3] - o1[0:3]).T).T], [k1[0:3]]])
    j3 = np.block([[np.cross(k2[0:3].T, (o4[0:3] - o2[0:3]).T).T], [k2[0:3]]])
    j4 = np.block([[np.cross(k3[0:3].T, (o4[0:3] - o3[0:3]).T).T], [k3[0:3]]])

    jac = np.block([j1, j2, j3, j4])

    return [trans_0to4, jac]

