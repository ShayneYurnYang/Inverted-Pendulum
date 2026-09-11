import numpy as np

# Physical Parameters
CART_MASS = 1.0       # kg
PENDULUM_MASS = 0.1   # kg
POLE_LENGTH = 0.4     # m
GRAVITY = 9.81        # m/s^2

# Initial State: [x, x_dot, theta, theta_dot]
INITIAL_STATE = [0.0, 0.0, np.pi / 12, 0.0]  # 15 degrees initial tilt

# Visual & Rendering Settings
WIDTH, HEIGHT = 800, 600
SCALE = 500           # 1 meter = 500 pixels
ORIGIN_X = WIDTH // 2
ORIGIN_Y = HEIGHT // 2 + 100
FPS = 30

# Colors (RGB tuples)
COLOR_BG = (0, 0, 0)
COLOR_TRACK = (255, 255, 255)
COLOR_CART = (0, 0, 255)
COLOR_POLE = (255, 0, 0)    