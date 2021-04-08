
import numpy as np
from scipy.linalg import expm


#  ###############################################################################################################
# Calculates and returns the homogenous transformation matrix
# for the given Denavit Hartenberg parameters
# inputs: thetai, displacement_di, displacement_ai, alphai (DH parameters by definition in rad)
# output: Numpy array of the Transformation matrix
#  ###############################################################################################################
def dh_homog(ti, di, ai, alphai):

    k_cross = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 0.]])
    i_cross = np.array([[0., 0., 0.], [0., 0., -1.], [0., 1., 0.]])

    exp_k = expm(ti * k_cross)
    exp_alphai = expm(alphai * i_cross)
    zero_vec = np.array([[0.], [0.], [0.]])

    angle = np.block([[exp_k, zero_vec], [0., 0., 0., 1.]])

    offset = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., di], [0., 0., 0., 1.]])
    length = np.array([[1., 0., 0., ai], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])

    twist = np.block([[exp_alphai, zero_vec], [0., 0., 0., 1.]])

    trans_matrix = angle @ offset @ length @ twist

    return trans_matrix
