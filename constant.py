#  ###############################################################################################################
# Document full of constants used in the control code
#
#
#  ###############################################################################################################
import numpy as np

# measured in mm
ARM_HEIGHT_DISPLACEMENT = 100.  # Lowest Resting Height of the Camera Tracker above the Body Tracker
SCISSOR_DISPLACEMENT = 30.      # Horizontal distance between body/head trackers at closest position to the arm

# a hard limit on the number of steps the manipulator PID will do
# this is mostly for testing, but gives insight to if the robot is stuck
PID_STEP_LIMIT = 1000

# if using webots simulation, this flag should be set to true, else set to false for real robot
using_webots = True

# if using the head extension functionality set to 1, else set to 0
using_head_ext = 1

# set to True if you want to plot the PID response on a graph, else False
plotPID = True

# proportional control variables along the diagonal
# (multiplies: [arm_displacement, head_displacement, pan, tilt] @ KP)
KP = np.array([[.015, 0., 0., 0.],                   # 1 arm_displacement KP
               [0., using_head_ext * .010, 0., 0.],  # 2 head_displacement KP
               [0., 0., .03, 0.],                    # 3 pan KP
               [0., 0., 0., .05]])                   # 4 tilt KP

# integral control variables along the diagonal
# (multiplies: [arm_displacement, head_displacement, pan, tilt] @ KI)
KI = np.array([[.000001, 0., 0., 0.],                # 1 arm_displacement KI
               [0., using_head_ext * .000, 0., 0.],  # 2 head_displacement KI
               [0., 0., 0.000, 0.],                  # 3 pan KI
               [0., 0., 0., 0.000001]])              # 4 tilt KI

# differential control variables along the diagonal
# (multiplies: [arm_displacement, head_displacement, pan, tilt] @ KD)
KD = np.array([[.014, 0., 0., 0.],                     # 1 arm_displacement KD
               [0., using_head_ext * .0095, 0., 0.],   # 2 head_displacement KD
               [0., 0., 0.029, 0.],                    # 3 pan KD
               [0., 0., 0., 0.049]])                   # 4 tilt KD

# threshold defining the acceptable joint errors of the manipulator (angle=rads, position=m)
angle_threshold = 0.02
position_threshold = 0.02

# Sets how many data points must be with the threshold to ensure the manipulator has stabilized
consecutive_manipulator_successes = 30

# planned trajectory variables for the manipulator (UNUSED)
TRAJECTORY_TIME = 1.            # trajectory time in seconds
TRAJECTORY_TIME_STEPS = 200     # number of time steps in the trajectory


