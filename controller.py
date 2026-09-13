import numpy as np
import scipy.linalg

import config


def compute_lqr_gain():
    M = config.CART_MASS
    m1, m2 = config.POLE_MASS_1, config.POLE_MASS_2
    l1, l2 = config.POLE_LEN_1, config.POLE_LEN_2
    g = config.GRAVITY

    I1 = (1.0 / 12.0) * m1 * (l1**2)
    I2 = (1.0 / 12.0) * m2 * (l2**2)
    lc1, lc2 = l1 / 2.0, l2 / 2.0

    M0 = np.array([
        [M + m1 + m2, m1 * lc1 + m2 * l1, m2 * lc2],
        [m1 * lc1 + m2 * l1, I1 + m1 * (lc1**2) + m2 * (l1**2), m2 * lc2 * l1],
        [m2 * lc2, m2 * lc2 * l1, I2 + m2 * (lc2**2)]
    ])
    
    inv_M0 = np.linalg.inv(M0)

    K_grav = np.array([
        [0, 0, 0],
        [0, (m1 * lc1 + m2 * l1) * g, 0],
        [0, 0, m2 * lc2 * g]
    ])

    A_acc = inv_M0 @ K_grav

    A = np.zeros((6, 6))
    A[0, 1] = 1.0
    A[2, 3] = 1.0
    A[4, 5] = 1.0
    
    A[1, 2], A[1, 4] = A_acc[0, 1], A_acc[0, 2]
    A[3, 2], A[3, 4] = A_acc[1, 1], A_acc[1, 2]
    A[5, 2], A[5, 4] = A_acc[2, 1], A_acc[2, 2]

    B_col = inv_M0 @ np.array([1.0, 0.0, 0.0])
    B = np.zeros((6, 1))
    B[1, 0] = B_col[0]
    B[3, 0] = B_col[1]
    B[5, 0] = B_col[2]

    Q = np.diag([
        30.0,    # x position penalty
        3.0,     # x velocity penalty
        400.0,   # theta1 angle penalty
        20.0,    # theta1 angular velocity penalty (dampens fast swinging)
        600.0,   # theta2 angle penalty
        30.0     # theta2 angular velocity penalty
    ])

    # Penalize large control inputs.
    R = np.array([[0.05]])


    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

K = compute_lqr_gain()

def compute_control_force(state):
    x = np.array(state)
    force_u = -(K @ x).item()
    return float(np.clip(force_u, -config.MAX_FORCE, config.MAX_FORCE))