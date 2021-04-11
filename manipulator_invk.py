
import numpy as np
import math
import constant
from kahan_p3 import kahan_p3
from kahan_p2 import kahan_p2


#  ###############################################################################################################
# calculates the inverse kinematics of the arm and head
# inputs: the origin, k_vector (approach) and i_vector (slide) of both the body and head
# outputs: Numpy array of the vertical displacement (mm), head extension (mm),
#          and the pan/tilt angles (rad)
#
# home position and bounds: arm_extension   = 0 = head at the bottom of the arm    bound [0,MAX_ARM_HEIGHT](bottom, top)
#                           head_extension  = 0 = head closest to arm              bound [0,MAX_HEAD_EXT] (in, out)
#                           pan             = 0 = camera facing left (portside)    bound [-pi/2, pi/2] -> (front, back)
#                           tilt            = 0 = camera straight down             bound [0, pi] -> (down, up)
#  ###############################################################################################################
def manipulator_invk(o_body, frame_body, o_head, frame_head):

    # define global vectors
    k_vec = np.array([[0], [0], [1]])
    i_vec = np.array([[1], [0], [0]])

    # define approach and slide vectors from frames
    k_head = frame_head @ k_vec
    k_body = frame_body @ k_vec
    i_head = frame_head @ i_vec
    i_body = frame_body @ i_vec

    # delta origin
    delta_o = (o_head - o_body)

    # arm joint = delta_origin_z - default_height
    arm_ext = delta_o[2, 0] - constant.ARM_HEIGHT_DISPLACEMENT

    # head joint = delta_origin_without_z - default_offset
    if abs(delta_o[0, 0]) < 1e-9 and abs(delta_o[1, 0]) < 1e-9:
        # avoid the norm function from dividing by zero
        head_ext = 0
    else:
        # head joint = delta_origin_without_z - default_offset
        head_ext = np.linalg.norm(delta_o - [[0.], [0.], delta_o[2]]) - constant.HEAD_DISPLACEMENT

    # if k_head and k_body are the same direction -> tilt angle is pi/2
    if abs(np.vdot(k_head, k_body) - 1.) < 1e-9:
        phi1 = kahan_p2(k_body, i_body, i_head)
        pan = phi1 - math.pi / 2.
        tilt = math.pi/2
    else:
        # calculate possible pan head angles
        [[theta1, phi1], [theta2, phi2]] = kahan_p3(k_head, k_body, i_head, i_body)

        # screen the correct phi (change based on positive/ negative tilt angles)
        if abs(phi1) < abs(phi2):
            pan = (phi2 + math.pi) % (2 * math.pi)
        else:
            pan = (phi1 + math.pi) % (2 * math.pi)

        # bound between [-pi, pi]
        if pan >= math.pi:
            pan -= 2 * math.pi

        # tilt = pi - angle_from_global_vertical
        tilt = math.pi - math.acos(np.vdot(k_body, i_head))

    return np.array([[arm_ext], [head_ext], [pan], [tilt]])


