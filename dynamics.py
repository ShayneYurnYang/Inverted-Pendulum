# dynamics.py

import numpy as np
from scipy.integrate import solve_ivp
import config

def pendulum_dynamics(t, state, force_u=0.0):
    x, x_dot, theta, theta_dot = state
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    total_mass = config.CART_MASS + config.PENDULUM_MASS

    # Cart acceleration including control input
    temp = (
        config.PENDULUM_MASS * config.POLE_LENGTH * (theta_dot**2) * sin_theta
        + config.PENDULUM_MASS * config.GRAVITY * sin_theta * cos_theta
        + force_u
    )
    x_ddot = temp / (total_mass - config.PENDULUM_MASS * (cos_theta**2))

    # Pendulum angular acceleration
    theta_ddot = (config.GRAVITY * sin_theta - x_ddot * cos_theta) / config.POLE_LENGTH

    return [x_dot, x_ddot, theta_dot, theta_ddot]

def step_physics(state, dt, force_u=0.0):
    if dt <= 0:
        return state

    sol = solve_ivp(
        fun=lambda t, y: pendulum_dynamics(t, y, force_u),
        t_span=(0, dt),
        y0=state,
        method="RK45",
    )
    
    # Extract calculated end state
    new_state = sol.y[:, -1]

    # Clip velocities to maximum limits
    new_state[1] = np.clip(new_state[1], -config.MAX_CART_VELOCITY, config.MAX_CART_VELOCITY)
    new_state[3] = np.clip(new_state[3], -config.MAX_POLE_VELOCITY, config.MAX_POLE_VELOCITY)

    return new_state