
import numpy as np
from kahan_p2 import kahan_p2


#  ###############################################################################################################
# Kahan problem: solves the equation exp(theta * s_hat cross) * u_hat = w_hat = exp(phi * t_hat cross) * v_hat
# inputs: s, t, u, v vectors (np.array)
# outputs: theta[0:1] and phi[0:1]   (index[0] = associated with positive, index[1] ~ negative)
#  ###############################################################################################################
def kahan_p3(s, t, u, v):

    # normalize vectors
    s_hat = s.T / np.linalg.norm(s)
    t_hat = t.T / np.linalg.norm(t)
    u_hat = u.T / np.linalg.norm(u)
    v_hat = v.T / np.linalg.norm(v)

    a_mat = [[1.0, np.vdot(t_hat, s_hat)], [np.vdot(s_hat, t_hat), 1.0]]
    b_vec = [[np.vdot(u_hat, s_hat)], [np.vdot(v_hat, t_hat)]]

    x_vec = np.linalg.solve(a_mat, b_vec)
    z_vec = x_vec[0] * s_hat + x_vec[1] * t_hat
    z_mag = np.linalg.norm(z_vec)

    if z_mag < 1:
        # Calc intermediate vectors
        w_hat1 = z_vec + np.sqrt(1 - z_mag ** 2) * np.cross(s_hat, t_hat) / np.linalg.norm(np.cross(s_hat, t_hat))
        w_hat2 = z_vec - np.sqrt(1 - z_mag ** 2) * np.cross(s_hat, t_hat) / np.linalg.norm(np.cross(s_hat, t_hat))

        # Calc results
        theta1 = kahan_p2(s_hat.T, u_hat.T, w_hat1.T)
        phi1 = kahan_p2(t_hat.T, v_hat.T, w_hat1.T)
        theta2 = kahan_p2(s_hat.T, u_hat.T, w_hat2.T)
        phi2 = kahan_p2(t_hat.T, v_hat.T, w_hat2.T)

        return [[theta1, phi1], [theta2, phi2]]

