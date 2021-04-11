

# unfinished and untested implementation due to time restraints
# calculates the required joint variable positions, velocities and accelerations to move
# the arm along a straight line trajectory
import numpy as np
from viscid.rotation import rot2axang
import constant
import math
from manipulator_invk import manipulator_invk
from manipulator_fwdk import manipulator_fwdk
import matplotlib.pyplot as plt


def arm_trajectory(od_body, cd_body, oc_body, cc_body, od_head, cd_head, oc_head, cc_head):

    # pre allocation
    jac = np.zeros((6, 4 * constant.TRAJECTORY_TIME_STEPS))     # 6x4 Jacobians
    oi = np.zeros((3, constant.TRAJECTORY_TIME_STEPS))          # 3x1 origins
    q = np.zeros((4, constant.TRAJECTORY_TIME_STEPS))           # 4x1 joint vars
    qdot = np.zeros((4, constant.TRAJECTORY_TIME_STEPS))        # 4x1 joint velocities
    qdotdot = np.zeros((4, constant.TRAJECTORY_TIME_STEPS))     # 4x1 joint accelerations

    # calc origin and frame changes relative to the body
    # desired origin
    od = od_head - od_body
    # current origin
    oc = oc_head - oc_body
    # desired frame
    cd = np.linalg.solve(cd_body, cd_head)
    # current frame
    cc = np.linalg.solve(cc_body, cc_head)

    # direction vector
    direction = (od - oc)/np.linalg.norm(od - oc)
    # axis/angle representation of the frame change
    angle = rot2axang(cd @ cc.T)

    # linear acceleration = 2 * half the distance / ( half the time )^2    (from dx = vi*t +.5a*t^2)
    acceleration = 2 * 0.5 * np.linalg.norm(od - oc) / (constant.TRAJECTORY_TIME / 2) ** 2
    wdot = angle[3] / (constant.TRAJECTORY_TIME / 2) ** 2

    # distance(/angle) from origin at step i
    di = .5 * acceleration * (1/constant.TRAJECTORY_TIME_STEPS) ** 2
    thetai = .5 * wdot * (1/constant.TRAJECTORY_TIME_STEPS) ** 2

    # calc first origin change and jacobian from current input
    oi[:, 0] = (oc + direction * di).T
    q0 = manipulator_invk(oc_body, cc_body, oc_head, cc_head)
    [temp, jac[:, 0:4]] = manipulator_fwdk([q0[0, 0], q0[1, 0], q0[2, 0], q0[3, 0]])

    # q(desired) = q(current) + Jinv * xdot
    # [[od-oc], [theta*axis]] = xdot
    xdot = np.block([[oi[0, 0] - oc[0]], [oi[1, 0] - oc[1]], [oi[2, 0] - oc[2]], [thetai * angle[0]],
                     [thetai * angle[1]], [thetai * angle[2]]])
    q[:, 0] = (q0 + np.linalg.pinv(jac[:, 0:4]) @ xdot).T

    # first half, constant acceleration
    for n in range(1, constant.TRAJECTORY_TIME_STEPS // 2):
        # distance(/angle) from origin at step i
        # di = vi*t + .5*a*t^2
        di = .5 * acceleration * ((n+1) / constant.TRAJECTORY_TIME_STEPS) ** 2
        thetai = .5 * wdot * ((n+1) / constant.TRAJECTORY_TIME_STEPS) ** 2
        thetai_plus1 = .5 * wdot * ((n+2) / constant.TRAJECTORY_TIME_STEPS) ** 2

        # origin at this time step
        oi[:, n] = (oc + direction * di).T

        # calc Jacobian + pseudo inverse
        [temp, jac[:, n*4: (n+1)*4]] = manipulator_fwdk(q[:, n-1])
        jinv = np.linalg.pinv(jac[:, n*4: (n+1)*4])

        # q(desired) = q(current) + Jinv * [[od-oc], [theta*axis]]
        xdot = np.block([[oi[0, n] - oi[0, n-1]], [oi[1, n] - oi[1, n-1]], [oi[2, n] - oi[2, n-1]],
                         [(thetai_plus1-thetai) * angle[0]], [(thetai_plus1-thetai) * angle[1]],
                         [(thetai_plus1-thetai) * angle[2]]])
        q[:, n] = (q[:, n-1].reshape(4, 1) + jinv @ xdot).T

        # qdot = jinv * vn
        qdot[:, n] = (jinv @ xdot * constant.TRAJECTORY_TIME_STEPS).T

        # define vdot and jdot
        vdot = np.block([[acceleration * direction], [wdot * angle[0]], [wdot * angle[1]], [wdot * angle[2]]])
        jdot = (jac[:, n*4: (n+1)*4] - jac[:, (n-1)*4: n*4]) * constant.TRAJECTORY_TIME_STEPS

        # qdotdot = Jinv * [vdot - Jdot * qdot]
        qdotdot[:, n] = (jinv @ (vdot - jdot @ qdot[:, n].reshape(4, 1))).T

    # second half
    print('second half trajectory')
    # calc peak velocity and angular rate
    vmax = acceleration * constant.TRAJECTORY_TIME / 2  # vf = vi + a*t
    wmax = wdot * constant.TRAJECTORY_TIME / 2

    # reverse acceleration
    acceleration = -acceleration
    wdot = -wdot

    for n in range(constant.TRAJECTORY_TIME_STEPS // 2, constant.TRAJECTORY_TIME_STEPS):
        # distance(/angle) from origin at step i
        # di = vi*t + .5*a*t^2
        di = vmax*(n-99)/constant.TRAJECTORY_TIME_STEPS + .5 * acceleration * (
                (n - 99) / constant.TRAJECTORY_TIME_STEPS) ** 2
        thetai = wmax*(n-99)/constant.TRAJECTORY_TIME_STEPS + .5 * wdot * (
                (n + 1) / constant.TRAJECTORY_TIME_STEPS) ** 2
        thetai_plus1 = wmax*(n-98)/constant.TRAJECTORY_TIME_STEPS + .5 * wdot * (
                (n + 2) / constant.TRAJECTORY_TIME_STEPS) ** 2

        # origin at this time step
        oi[:, n] = (oi[:, constant.TRAJECTORY_TIME_STEPS // 2 - 1].reshape(3, 1) + direction * di).T

        # calc Jacobian + pseudo inverse
        [temp, jac[:, n * 4: (n + 1) * 4]] = manipulator_fwdk(q[:, n - 1])
        jinv = np.linalg.pinv(jac[:, n * 4: (n + 1) * 4])

        # q(desired) = q(current) + Jinv * [[od-oc], [theta*axis]]
        xdot = np.block([[oi[0, n] - oi[0, n - 1]], [oi[1, n] - oi[1, n - 1]], [oi[2, n] - oi[2, n - 1]],
                         [(thetai_plus1 - thetai) * angle[0]], [(thetai_plus1 - thetai) * angle[1]],
                         [(thetai_plus1 - thetai) * angle[2]]])
        q[:, n] = (q[:, n - 1].reshape(4, 1) + jinv @ xdot).T

        # qdot = jinv * vn
        qdot[:, n] = (jinv @ xdot * constant.TRAJECTORY_TIME_STEPS).T

        # define vdot and jdot
        vdot = np.block([[acceleration * direction], [wdot * angle[0]], [wdot * angle[1]], [wdot * angle[2]]])
        jdot = (jac[:, n * 4: (n + 1) * 4] - jac[:, (n - 1) * 4: n * 4]) * constant.TRAJECTORY_TIME_STEPS

        # qdotdot = Jinv * [vdot - Jdot * qdot]
        qdotdot[:, n] = (jinv @ (vdot - jdot @ qdot[:, n].reshape(4, 1))).T

    # plot
    print('plot trajectory')
    fig, axs = plt.subplots(2, 2)
    time = [i for i in range(constant.TRAJECTORY_TIME_STEPS)]
    axs[0, 0].set_title('Arm Displacement')
    axs[0, 0].plot(time[:], q[0, :])
    axs[0, 1].set_title('Head Extension')
    axs[0, 1].plot(time[:], q[1, :])
    axs[1, 0].set_title('Pan Angle')
    axs[1, 0].plot(time[:], q[2, :] * 180. / math.pi)
    axs[1, 1].set_title('Tilt Angle')
    axs[1, 1].plot(time[:], q[3, :] * 180. / math.pi)
    plt.show()

    fig, axs = plt.subplots(2, 2)
    time = [i for i in range(constant.TRAJECTORY_TIME_STEPS)]
    axs[0, 0].set_title('hdot')
    axs[0, 0].plot(time[:], qdot[0, :])
    axs[0, 1].set_title('xdot')
    axs[0, 1].plot(time[:], qdot[1, :])
    axs[1, 0].set_title('Pandot')
    axs[1, 0].plot(time[:], qdot[2, :] * 180. / math.pi)
    axs[1, 1].set_title('Tiltdot')
    axs[1, 1].plot(time[:], qdot[3, :] * 180. / math.pi)
    plt.show()

    fig, axs = plt.subplots(2, 2)
    time = [i for i in range(constant.TRAJECTORY_TIME_STEPS)]
    axs[0, 0].set_title('hdotdot')
    axs[0, 0].plot(time[:], qdotdot[0, :])
    axs[0, 1].set_title('xdotdot')
    axs[0, 1].plot(time[:], qdotdot[1, :])
    axs[1, 0].set_title('Pandotdot')
    axs[1, 0].plot(time[:], qdotdot[2, :] * 180. / math.pi)
    axs[1, 1].set_title('Tiltdotdot')
    axs[1, 1].plot(time[:], qdotdot[3, :] * 180. / math.pi)
    plt.show()



    fig2, ax2 = plt.subplots(2, 1)
    distance = np.zeros(constant.TRAJECTORY_TIME_STEPS)
    for n in range(constant.TRAJECTORY_TIME_STEPS):
        distance[n] = np.linalg.norm(od_head-oi[:, n].reshape(3, 1))
    ax2[0].plot(time[:], distance[:])
    plt.show()
    [trans_0to4, jac] = manipulator_fwdk(q[:, constant.TRAJECTORY_TIME_STEPS - 1])
    return [trans_0to4, jac]
