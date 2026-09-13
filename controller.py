import numpy as np
import scipy.linalg
import config

def compute_lqr_gain():
    M = config.CART_MASS
    m = config.PENDULUM_MASS
    l = config.POLE_LENGTH
    g = config.GRAVITY

    # Correct linearized matrices matching dynamics.py
    A = np.array([
        [0, 1, 0, 0],
        [0, 0, -(m * g) / M, 0],
        [0, 0, 0, 1],
        [0, 0, ((M + m) * g) / (M * l), 0]
    ])

    B = np.array([
        [0],
        [1 / M],
        [0],
        [-1 / (M * l)]  # Negative sign here: positive force causes negative angular accel
    ])

    # Q penalizes [x, x_dot, theta, theta_dot]
    Q = np.diag([20.0, 15.0, 100.0, 30.0])
    R = np.array([[0.1]])

    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

K = compute_lqr_gain()

def compute_control_force(state):
    x = np.array(state)
    
    # Calculate force u = -K * x
    force_u = -(K @ x).item()
    
    # Optional: Clip force to prevent numeric explosions if pushed too far
    return float(np.clip(force_u, -300.0, 300.0))