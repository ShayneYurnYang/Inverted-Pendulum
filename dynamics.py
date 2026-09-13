import numpy as np
from scipy.integrate import solve_ivp

import config


def pendulum_dynamics(t, state, force_u=0.0):
    x, x_dot, t1, t1_dot, t2, t2_dot = state

    M = config.CART_MASS
    m1, m2 = config.POLE_MASS_1, config.POLE_MASS_2
    l1, l2 = config.POLE_LEN_1, config.POLE_LEN_2
    g = config.GRAVITY

    # Rod inertia about each center of mass.
    I1 = (1.0 / 12.0) * m1 * (l1**2)
    I2 = (1.0 / 12.0) * m2 * (l2**2)

    # Center-of-mass distances.
    lc1 = l1 / 2.0
    lc2 = l2 / 2.0

    d1 = m1 + m2
    d2 = m2 * lc2
    d3 = m1 * lc1 + m2 * l1

    # Mass matrix.
    M_mat = np.array([
        [M + m1 + m2, d3 * np.cos(t1), d2 * np.cos(t2)],
        [d3 * np.cos(t1), I1 + m1 * (lc1**2) + m2 * (l1**2), d2 * l1 * np.cos(t1 - t2)],
        [d2 * np.cos(t2), d2 * l1 * np.cos(t1 - t2), I2 + m2 * (lc2**2)]
    ])

    # Forces and gravity.
    RHS = np.array([
        force_u + d3 * (t1_dot**2) * np.sin(t1) + d2 * (t2_dot**2) * np.sin(t2),
        d3 * g * np.sin(t1) - d2 * l1 * (t2_dot**2) * np.sin(t1 - t2),
        d2 * g * np.sin(t2) + d2 * l1 * (t1_dot**2) * np.sin(t1 - t2)
    ])

    # Fall back to a pseudo-inverse if the matrix is singular.
    try:
        accelerations = np.linalg.solve(M_mat, RHS)
    except np.linalg.LinAlgError:
        accelerations = np.linalg.pinv(M_mat) @ RHS

    return [x_dot, accelerations[0], t1_dot, accelerations[1], t2_dot, accelerations[2]]

def step_physics(state, dt, force_u=0.0):
    if dt <= 0:
        return state
    sol = solve_ivp(
        fun=lambda t, y: pendulum_dynamics(t, y, force_u),
        t_span=(0, dt),
        y0=state,
        method="RK45",
    )
    return sol.y[:, -1]