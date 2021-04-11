#  ###############################################################################################################
# Document full of constants used in the control code
#
#
#  ###############################################################################################################
import numpy as np

# measured in mm
ARM_HEIGHT_DISPLACEMENT = 100.  # Lowest Resting Height of the Camera Tracker above the Body Tracker
HEAD_DISPLACEMENT = 30.      # Horizontal distance between body/head trackers at closest position to the arm

DATA_FREQUENCY_HZ = 120         # the rate the robot receives updates from the vive trackers

# a hard limit on the number of steps the manipulator PID will do
# this is mostly for testing, but gives insight to if the robot is stuck
PID_STEP_LIMIT = DATA_FREQUENCY_HZ * 15     # 15 seconds

# if using webots simulation, this flag should be set to true, else set to false for real robot
using_webots = True

# if using the head extension functionality set to 1, else set to 0
using_head_ext = 1

# set to True if you want to plot the PID response on a graph, else False
plotPID = True

# proportional control variables along the diagonal
# (multiplies: [arm_displacement, head_displacement, pan, tilt] @ KP)
KP = np.array([[.055, 0., 0., 0.],                   # 1 arm_displacement KP
               [0., using_head_ext * .010, 0., 0.],  # 2 head_displacement KP
               [0., 0., .03, 0.],                    # 3 pan KP
               [0., 0., 0., .05]])                   # 4 tilt KP

# integral control variables along the diagonal
# (multiplies: [arm_displacement, head_displacement, pan, tilt] @ KI)
KI = np.array([[.00000, 0., 0., 0.],                # 1 arm_displacement KI
               [0., using_head_ext * .000, 0., 0.],  # 2 head_displacement KI
               [0., 0., 0.000, 0.],                  # 3 pan KI
               [0., 0., 0., 0.000001]])              # 4 tilt KI

# differential control variables along the diagonal
# (multiplies: [arm_displacement, head_displacement, pan, tilt] @ KD)
KD = np.array([[.05, 0., 0., 0.],                     # 1 arm_displacement KD
               [0., using_head_ext * .0095, 0., 0.],   # 2 head_displacement KD
               [0., 0., 0.029, 0.],                    # 3 pan KD
               [0., 0., 0., 0.049]])                   # 4 tilt KD

# threshold defining the acceptable joint errors of the manipulator (angle=rads, position=mm)
angle_threshold = 0.02
position_threshold = 2

# Sets how many data points must be with the threshold to ensure the manipulator has stabilized
consecutive_manipulator_successes = 60      # total time = # successes / frequency of data

# maximum workspace x_dimension distance from center (mm)
x_room_distance_from_center = 3000.
# maximum workspace y_dimension distance from center (mm)
y_room_distance_from_center = 3000.

# obstacle array discretization size: search algorithm will use a X by Y array
OBSTACLE_GRID_SIZE_X = 100  # even values only
OBSTACLE_GRID_SIZE_Y = 100  # even values only

# integer dividing an x,y coordinate by these will give its coordinate in the search array
X_ROOM_DISCRETIZATION = x_room_distance_from_center // (OBSTACLE_GRID_SIZE_X // 2)
Y_ROOM_DISCRETIZATION = y_room_distance_from_center // (OBSTACLE_GRID_SIZE_Y // 2)

# test obstacle grid
EXAMPLE_TEST_GRID = np.array([
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])

# planned trajectory variables for the manipulator (UNUSED)
TRAJECTORY_TIME = 1.            # trajectory time in seconds
TRAJECTORY_TIME_STEPS = 200     # number of time steps in the trajectory




