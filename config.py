import numpy as np
# Physical Limits
MAX_CART_VELOCITY = 3.0      # m/s
MAX_POLE_VELOCITY = 10.0     # rad/s (~573 deg/s)

# Physical Parameters
CART_MASS = 1.0       # kg
PENDULUM_MASS = 0.1   # kg
POLE_LENGTH = 0.4     # m
GRAVITY = 9.81        # m/s^2

# Initial State: [x, x_dot, theta, theta_dot]
# config.py

INITIAL_STATE = [0.0, 0.0, np.pi - np.deg2rad(175), 0.0]  # Start at 175 degrees from bottom

# Visual & Rendering Settings
WIDTH, HEIGHT = 800, 600
SCALE = 150  # 1 meter = 150 pixels (gives ~2.6 meters of track space)
ORIGIN_X = WIDTH // 2
ORIGIN_Y = HEIGHT // 2 + 100
FPS = 60

# Colors (RGB tuples)
COLOR_BG = (0, 0, 0)
COLOR_TRACK = (255, 255, 255)
COLOR_CART = (0, 0, 255)
COLOR_POLE = (255, 0, 0)    