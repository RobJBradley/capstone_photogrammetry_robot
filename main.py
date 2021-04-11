

import numpy as np
import math
import A_star
import body_control

from manipulator_fwdk import manipulator_fwdk
from manipulator_invk import manipulator_invk
from arm_trajectory import arm_trajectory
from manipulator_control import manipulator_seek


def test_manipulator_pid():
    print('test manipulator PID')
    seek_var = [1400., 138.4, -1.3, math.pi/2]
    [trans04, temp] = manipulator_fwdk(seek_var)
    xyz = trans04[3, 0:3]
    zero = np.zeros((3, 1))
    eye = np.eye(3)
    # print('od:', xyz)
    # print('cd:', trans04[0:3, 0:3])
    manipulator_seek(zero, eye, xyz, trans04[0:3, 0:3])


def test_manipulator_kinematics():
    print("kinematics test:")
    seek_var = [1.0, 0.35, -73. * math.pi/180, 80. * math.pi/180]
    [trans04, temp] = manipulator_fwdk(seek_var)  # [1.42, .25, math.pi * 5 / 16, math.pi * 3 / 4]
    xyz = trans04 @ np.array([[0], [0], [0], [1]])
    zero = np.zeros((3, 1))
    eye = np.eye(3)
    joints = manipulator_invk(zero, eye, xyz[0:3], trans04[0:3, 0:3])
    [real_trans, temp2] = manipulator_fwdk([joints[0, 0], joints[1, 0], joints[2, 0], joints[3, 0]])
    xyz = real_trans @ np.array([[0], [0], [0], [1]])
    test_vec = np.array([[1], [0], [0]]).reshape(3, 1)
    direction = real_trans[0:3, 0:3] @ test_vec
    print("testing:  ", seek_var)
    print("joints:   ", joints.T)
    print("direction:", direction.T)
    print(np.linalg.norm(direction))


def test_manipulator_trajectory():
    print('test trajectory')
    [trans04, temp] = manipulator_fwdk([1.42, .25, math.pi / 2, math.pi * 3 / 4])
    xyz = trans04 @ np.array([[0], [0], [0], [1]])
    zero = np.zeros((3, 1))
    eye = np.eye(3)
    # print('od:', xyz[0:3])
    # print('cd:', trans04[0:3, 0:3])
    [trans04_soln, temp] = arm_trajectory(zero, eye, zero, eye, xyz[0:3], trans04[0:3, 0:3], zero, eye)
    xyz_soln = trans04_soln @ np.array([[0], [0], [0], [1]])
    print('on - od:', xyz_soln[0:3] - xyz[0:3])
    print('cn - cd:', trans04_soln[0:3, 0:3] - trans04[0:3, 0:3])


def test_obj_discretization():
    # test obj discretization
    vertices = [[1, 1, 1], [439.6, 888.8, 9], [-264, 666, -981], [-2000, -2000, 0], [-2000, -2000 + 80, 0],
                [-1999, -1999, 0], [1999, 1999, 0], [-1999 + 80, 1999, 0], [1999 - 80, -1999 + 80, 0]]
    body_control.discretize_obj_vertices(vertices)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # test_manipulator_trajectory()

    # test_manipulator_kinematics()

    # test_manipulator_pid()

    # A_star.test_a_star()

    test_obj_discretization()









