
import numpy as np
from arm_invk import arm_invk
from arm_fwdk import arm_fwdk
import matplotlib.pyplot as plt
import constant


#  ###############################################################################################################
# Function that controls the manipulator to seek a position
#
# inputs:
#           np array of the origin of the body (3x1)
#           np array of the frame of the body (3x3)         i = forward along robot body, k = upwards in space
#           np array of the origin of the head (3x1)
#           np array of the frame of the head (3x3)         i = forward along camera, k = upwards of camera
#
# result:
#           robot moves the manipulator to the desired location, accounting for where the body will be when
#           the body finishes its movement (the manipulator is smart and will go to the position it needs to be at
#           the end- without constantly changing targets as the body moves)
#  ###############################################################################################################
def manipulator_seek(od_body, cd_body, od_head, cd_head):

    # initialize errors
    prev_error = np.zeros((1, 4))
    sum_error = np.zeros((1, 4))
    motor_values = np.zeros((1, 4))

    # setup graphing parameters
    fig, axs = plt.subplots(2, 2)
    time = [i for i in range(constant.PID_STEP_LIMIT)]
    error_graph = np.zeros((4, constant.PID_STEP_LIMIT))

    # initialize iteration counters (stability checking indexes)
    counter = 0
    not_in_place = 0

    # this is initialization for TESTING ONLY #########################################################
    # initial arm position FOR TESTING
    q_current = np.array([[.8], [.35], [1.2], [.7]])

    # find target joint parameters
    q_target = arm_invk(od_body, cd_body, od_head, cd_head)

    print("Begin Manipulator Seeking Loop to:", q_target.T)
    # Loop until error is within the threshold
    while True:
        # get current positions and orientations from udp packet
        if constant.using_webots:
            [oc_body, cc_body, oc_head, cc_head] = get_webots_data(motor_values, q_current.T)
        else:
            [oc_body, cc_body, oc_head, cc_head] = get_robot_data()

        # current joint variables
        q_current = arm_invk(oc_body, cc_body, oc_head, cc_head)

        # calculate adjustments based on PID
        [motor_values_adjust, prev_error, sum_error] = manipulator_pid(q_current, q_target, prev_error, sum_error)

        # add adjustment to the previous motor PWM values
        motor_values = np.add(motor_values, motor_values_adjust)

        # limit motor PWM values to the range [-1,1]
        motor_values = np.array([max(min(motor_values[0, 0], 1.), -1.), max(min(motor_values[0, 1], 1.), -1.),
                                 max(min(motor_values[0, 2], 1.), -1.), max(min(motor_values[0, 3], 1.), -1.)])
        motor_values = motor_values.reshape(1, 4)

        # output motor values to the robot
        if constant.using_webots:
            send_to_webots(motor_values)
        else:
            send_to_robot(motor_values)

        # record the error to check for stability of the system
        error_graph[:, counter] = prev_error[:]

        # if any joint is not within acceptable threshold reset stability check
        for k in range(len(q_current)):
            # first 2 errors are joint displacements (setting using_head_ext=0 does not include it in the error)
            if (k <= constant.using_head_ext) and (abs(error_graph[k, counter]) > constant.position_threshold):
                not_in_place = 0
            # second 2 errors are angle displacements
            elif (k >= 2) and (abs(error_graph[k, counter]) > constant.angle_threshold):
                not_in_place = 0
            else:
                not_in_place += 1

        # increment the current number of steps (used to index previous errors to determine stability)
        counter += 1

        # if robot has been within the thresholds for a specified amount of time steps -> stability achieved!
        if not_in_place > constant.consecutive_manipulator_successes:
            print("Manipulator Seek Success")
            # if plotting the seek operation is enabled: plot each joint error
            if constant.plotPID:
                print("final joint error is:\n", prev_error)
                axs[0, 0].set_title('Arm Displacement')
                axs[0, 0].plot(time[0:counter], error_graph[0, 0:counter])
                axs[0, 1].set_title('Head Extension')
                axs[0, 1].plot(time[0:counter], error_graph[1, 0:counter])
                axs[1, 0].set_title('Pan Angle')
                axs[1, 0].plot(time[0:counter], error_graph[2, 0:counter])
                axs[1, 1].set_title('Tilt Angle')
                axs[1, 1].plot(time[0:counter], error_graph[3, 0:counter])
                plt.subplots_adjust(hspace=.32, wspace=.3)
                plt.show()
                [trans04, temp] = arm_fwdk([q_current[0, 0], q_current[1, 0], q_current[2, 0], q_current[3, 0]])
                xyz = trans04 @ np.array([[0], [0], [0], [1]])
                print("difference in origin:\n", np.subtract(od_head.T, xyz[0:3].T))
                print("difference in frame:\n", np.subtract(cd_head, trans04[0:3, 0:3]))
            break

        # timeout failsafe -> if a PID_STEP_LIMIT is reached something is likely wrong
        # (unstable request or impossible geometry)
        if counter >= constant.PID_STEP_LIMIT:
            print("MANIPULATOR SEEK ERROR: TOO MANY TIME STEPS")
            if constant.plotPID:
                print("ERROR: TOO MANY TIME STEPS")
                axs[0, 0].set_title('Arm Displacement')
                axs[0, 0].plot(time[:], error_graph[0, :])
                axs[0, 1].set_title('Head Extension')
                axs[0, 1].plot(time[:], error_graph[1, :])
                axs[1, 0].set_title('Pan Angle')
                axs[1, 0].plot(time[:], error_graph[2, :])
                axs[1, 1].set_title('Tilt Angle')
                axs[1, 1].plot(time[:], error_graph[3, :])
                plt.subplots_adjust(hspace=.32, wspace=.3)
                plt.show()
            return False
    return True


#  ###############################################################################################################
# function that runs the PID on each joint of the manipulator
# inputs:
#         np array of the current joint parameters
#         np array of the target joint parameters
#         np array of the previous error of each joint
#         np array of the total sum of errors of each joint
# outputs:
#         np array of the adjustment to each motor PID
#         np array of the current joint error
#         np array of the total sum of joint errors
#  ###############################################################################################################
def manipulator_pid(q_current, q_target, prev_error, sum_error):

    # error of [arm_ext, scissor_ext, pan, tilt]
    error = np.array([q_target[0] - q_current[0], q_target[1] - q_current[1],
                      q_target[2] - q_current[2], q_target[3] - q_current[3]]).T

    # calculate motor adjustment from the proportional, differential and integral constants
    motor_values_adjust = error @ constant.KP - prev_error @ constant.KD + sum_error @ constant.KI

    # sum the error for the integral term
    sum_error = np.add(sum_error, error)

    return [motor_values_adjust, error, sum_error]


# example json packet:
# {
#   body: {
#       ox: x_val
#       oy: y_val
#       oz: z_val
#       frame: quaternion
#   }
#   head: {
#       ox: x_val
#       oy: y_val
#       oz: z_val
#       frame: quaternion
#   }
#  ###############################################################################################################
# Function that waits for data from the Vive tracker, receives a json packet, and converts it into Numpy arrays
# input: none
# output:
#           np array of the origin of the body (3x1)
#           np array of the frame of the body (3x3)         i = forward along robot body, k = upwards in space
#           np array of the origin of the head (3x1)
#           np array of the frame of the head (3x3)         i = forward along camera, k = upwards of camera
#  ###############################################################################################################
def get_robot_data():

    # get json packet from udp

    # parse json packet

    # convert quaternion to rotation matrix

    # assign current origin (o)/frame (c) data for body/head
    [oc_body, cc_body, oc_head, cc_head] = [np.zeros((3, 1)), np.eye(3), np.zeros((3, 1)), np.eye(3)]

    return [oc_body, cc_body, oc_head, cc_head]


#  ###############################################################################################################
# Function that sends the manipulator PWM values to the microcontroller
# input:
#           np array of the motor values (1x4)  [arm_displacement_PWM, head_displacement_PWM, pan_PWM, tilt_PWM]
#                                               ranging from [0,1]
# result: the microcontroller is sent the PWM value of each joint motor for the manipulator
#  ###############################################################################################################
def send_to_robot(motor_values):

    # send motor values to the MCU

    return


#  ###############################################################################################################
# Function to receive webots data and convert it to the same data from the real system
# input: none
# output: the origin and frame of the body, and the origin and frame of the head
#             [oc_body, cc_body,                    oc_head, cc_head]
#  ###############################################################################################################
def get_webots_data(motor_values, joint_vars):
    test_speed_val = 1000
    # get joint data [arm_displacement, head_displacement, pan, tilt]
    joint_vars = [joint_vars[0, 0] + motor_values[0, 0]*test_speed_val/120.,
                  joint_vars[0, 1] + motor_values[0, 1]*test_speed_val/120.,
                  joint_vars[0, 2] + motor_values[0, 2]*test_speed_val/120.,
                  joint_vars[0, 3] + motor_values[0, 3]*test_speed_val/120.]
    # joint_vars = np.add([motor_values[0, 0] * 30 / 120., motor_values[0, 1] * 30 / 120.,
    #                      motor_values[0, 2] * 30 / 120., motor_values[0, 3] * 30 / 120.], joint_vars.T)
    # ############################################### need to set this
    # print(joint_vars)

    [trans, temp] = arm_fwdk(joint_vars)

    xyz = trans @ np.array([[0], [0], [0], [1]])

    # assign current origin (o)/frame (c) data for body/head    ################ for now body is stationary
    [oc_body, cc_body, oc_head, cc_head] = [np.zeros((3, 1)), np.eye(3), xyz[0:3], trans[0:3, 0:3]]

    return [oc_body, cc_body, oc_head, cc_head]


#  ###############################################################################################################
# Function that sends the manipulator PWM values to the webots simulation motors
# input:
#           np array of the motor values (1x4)  [arm_displacement_PWM, head_displacement_PWM, pan_PWM, tilt_PWM]
#                                               ranging from [0,1]
# result: sends the data to the webots simulation motors
#  ###############################################################################################################
def send_to_webots(motor_values):

    # send motor values to the MCU

    return
