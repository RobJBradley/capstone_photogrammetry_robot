# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
from vectors import Point, Vector
import numpy as np
from viscid.rotation import rot2axang
import math


from DH_homog import dh_homog
from arm_fwdk import arm_fwdk
from arm_invk import arm_invk
from kahan_p2 import kahan_p2
from arm_trajectory import arm_trajectory
from manipulator_control import manipulator_seek


def print_hi(name):
    v1 = Vector(1, 2, 3)  # => Vector(1, 2, 3)
    v2 = Vector(0, 0, 0)
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.
    components = [1.2, 3.4, 6]
    v1.from_list(components)
    print(f'{v1.magnitude()}')
    v2 = v1.multiply(1/v1.magnitude())
    print(f'{v2.magnitude()}')

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    # print(dh_homog(2.3, 1.2, 3.6, -1.1))
    # [t_mat, j_mat] = arm_fwdk([1.2, .9, .8, .1])
    # print('transformation matrix:')
    # print(t_mat)
    # print('jacobian:')
    # print(j_mat)
    # print(kahan_p2(np.array([[1.], [0.], [0.]]), np.array([[0.], [0.], [1.]]), np.array([[0.], [1.], [0.]])))
    # test_frame = np.array([[0., -1., 1.], [1., 0., 0.], [0., 0., 6.]])
    # print('test')
    # print(test_frame @ np.array([[0], [0], [1]]))
    # test = np.zeros((3, 3 * 4))
    # print(test)
    # test[0:3, 0:3] = np.array([[1, 2, 3], [1, 2, 3], [1, 2, 3]])
    # print(test)
    # print((3 * test_frame @ np.array([[0], [0], [1]])).T)
    # print(rot2axang(test_frame))

    # print('test trajectory')
    # [trans04, temp] = arm_fwdk([1.42, .25, math.pi/2, math.pi*3/4])
    # xyz = trans04 @ np.array([[0], [0], [0], [1]])
    # zero = np.zeros((3, 1))
    # eye = np.eye(3)
    # # print('od:', xyz[0:3])
    # # print('cd:', trans04[0:3, 0:3])
    # [trans04_soln, temp] = arm_trajectory(zero, eye, zero, eye, xyz[0:3], trans04[0:3, 0:3], zero, eye)
    # xyz_soln = trans04_soln @ np.array([[0], [0], [0], [1]])
    # print('on - od:', xyz_soln[0:3] - xyz[0:3])
    # print('cn - cd:', trans04_soln[0:3, 0:3] - trans04[0:3, 0:3])

    # soln = np.linalg.solve(test_frame, trans04[0:3, 0:3])
    # print(soln)
    # print(test_frame @ soln)
    # print(trans04[0:3, 0:3])

    # motor_values = np.array([.2, -2., 4., 0.5])
    # test = np.array([max(min(motor_values[0], 1), 0), max(min(motor_values[1], 1), 0),
    #                  max(min(motor_values[2], 1), 0), max(min(motor_values[3], 1), 0)])
    # print(test)
    # print(test[0])
    # multiplier = np.array([[2., 0., 0., 0.], [0., 0.5, 0., 0.], [0., 0., 2., 0.], [0., 0., 0., -4.]])
    # print(test @ multiplier)
    # print([test[0] * multiplier[0], test[1] * multiplier[1], test[2] * multiplier[2], test[3] * multiplier[3]])

    # print("kinematics test:")
    # seek_var = [1.0, 0.35, -73. * math.pi/180, 5. * math.pi/180]
    # [trans04, temp] = arm_fwdk(seek_var)  # [1.42, .25, math.pi * 5 / 16, math.pi * 3 / 4]
    # xyz = trans04 @ np.array([[0], [0], [0], [1]])
    # zero = np.zeros((3, 1))
    # eye = np.eye(3)
    # joints = arm_invk(zero, eye, xyz[0:3], trans04[0:3, 0:3])
    # [real_trans, temp2] = arm_fwdk([joints[0, 0], joints[1, 0], joints[2, 0], joints[3, 0]])
    # xyz = real_trans @ np.array([[0], [0], [0], [1]])
    # test_vec = np.array([[1], [0], [0]]).reshape(3, 1)
    # direction = real_trans[0:3, 0:3] @ test_vec
    # print("testing:  ", seek_var)
    # print("joints:   ", joints.T)
    # print("direction:", direction.T)
    # print(np.linalg.norm(direction))

    print('test manipulator PID')
    seek_var = [1.42, 0.1, -1.3, math.pi/2]
    [trans04, temp] = arm_fwdk(seek_var)  # [1.42, .25, math.pi * 5 / 16, math.pi * 3 / 4]
    xyz = trans04 @ np.array([[0], [0], [0], [1]])
    zero = np.zeros((3, 1))
    eye = np.eye(3)
    # print('od:', xyz[0:3])
    # print('cd:', trans04[0:3, 0:3])
    manipulator_seek(zero, eye, xyz[0:3], trans04[0:3, 0:3])









# See PyCharm help at https://www.jetbrains.com/help/pycharm/

