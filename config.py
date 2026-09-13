# Physical parameters.
CART_MASS = 1.0       # kg (M)
POLE_MASS_1 = 0.1     # kg (m1)
POLE_MASS_2 = 0.1     # kg (m2)
POLE_LEN_1 = 0.4      # m  (l1)
POLE_LEN_2 = 0.3      # m  (l2)
GRAVITY = 9.81        # m/s^2
MAX_FORCE = 200.0  # Maximum motor force in newtons.

# Initial state: x, x_dot, theta1, theta1_dot, theta2, theta2_dot.
INITIAL_STATE = [0.0, 0.0, 0.05, 0.0, 0.0, 0.0]

# Display settings.
WIDTH, HEIGHT = 800, 600
SCALE = 100
ORIGIN_X = WIDTH // 2
ORIGIN_Y = HEIGHT // 2 + 100
FPS = 60

COLOR_BG = (0, 0, 0)
COLOR_TRACK = (255, 255, 255)
COLOR_CART = (0, 0, 255)
COLOR_POLE1 = (255, 0, 0)
COLOR_POLE2 = (0, 255, 0)