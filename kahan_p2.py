
import numpy as np


#  ###############################################################################################################
# Kahan problem: solves the equation exp(theta * s_hat cross) * u_hat = v_hat
# inputs: s, u, v vectors (np.array)
# output: the angle theta to rotate u_hat about s_hat
#  ###############################################################################################################
def kahan_p2(s, u, v):

    # normalize vectors
    s_hat = s.T / np.linalg.norm(s)
    u_hat = u.T / np.linalg.norm(u)
    v_hat = v.T / np.linalg.norm(v)

    a = np.vdot(s_hat, u_hat)
    b = np.vdot(s_hat, v_hat)

    if abs(a - b) < 1e-3:
        numerator = np.linalg.norm(np.cross(s_hat, np.subtract(u_hat, v_hat)))
        denominator = np.linalg.norm(np.cross(s_hat, np.add(u_hat, v_hat)))
        # prevent dividing by zero
        if denominator == 0:
            denominator = 1e-9
        theta = 2 * np.arctan(numerator / denominator)
        sign = np.vdot(v_hat, np.cross(s_hat, np.subtract(u_hat, v_hat)))
        if sign < 0.:
            theta = -1. * theta
        return theta
    else:
        print('NaN')

