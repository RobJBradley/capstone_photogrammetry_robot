import numpy as np

# measured in mm
ARM_HEIGHT_DISPLACEMENT = 30.
SCISSOR_DISPLACEMENT = 100.  # Lowest Resting Height above Body Tracker

TRAJECTORY_TIME = 1.
TRAJECTORY_TIME_STEPS = 200
PID_STEP_LIMIT = 500

# if using webots simulation, this flag should be set to true, else set to false
using_webots = True

# if using the head extension functionality set to 1, else set to 0
using_head_ext = 1

# use if you want to plot the PID response
plotPID = True

# proportional control variables along the diagonal (multiplies: [arm_displacement, head_displacement, pan, tilt] @ KP)
KP = np.array([[.015, 0., 0., 0.], [0., using_head_ext * .010, 0., 0.], [0., 0., .03, 0.], [0., 0., 0., .05]])

# integral control variables along the diagonal (multiplies: [arm_displacement, head_displacement, pan, tilt] @ KI)
KI = np.array([[.000001, 0., 0., 0.], [0., using_head_ext * .000, 0., 0.], [0., 0., 0.000, 0.], [0., 0., 0., 0.000001]])

# differential control variables along the diagonal (multiplies: [arm_displacement, head_displacement, pan, tilt] @ KD)
KD = np.array([[.014, 0., 0., 0.], [0., using_head_ext * .0095, 0., 0.], [0., 0., 0.029, 0.], [0., 0., 0., 0.049]])

# threshold defining the acceptable joint errors (angle=rads, position=m)
angle_threshold = 0.02
position_threshold = 0.02

# Sets how many data points must be with threshold to ensure the manipulator has settled
consecutive_manipulator_successes = 30

# joint threshold that prevents normalizing at 0
ZERO_ALLOWANCE = 1e-2

