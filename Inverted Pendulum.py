import sys
import numpy as np
import pygame
from scipy.integrate import solve_ivp

# --- Physics Parameters ---
Cart_Mass = 1.0
Pendulum_Mass = 0.1
Pole_Length = 0.4  # meters
Gravity = 9.81

# --- Pygame Setup ---
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Real-Time Inverted Pendulum Simulation")
clock = pygame.time.Clock()

# Visual scaling: 1 meter = 500 pixels
SCALE = 500
ORIGIN_X = WIDTH // 2
ORIGIN_Y = HEIGHT // 2 + 100

# State: [x, x_dot, theta, theta_dot]
state = [0.0, 0.0, np.pi / 6, 0.0]  # Start tipped at 30 degrees


def pendulum_dynamics(t, state):
    x, x_dot, theta, theta_dot = state
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    total_mass = Cart_Mass + Pendulum_Mass

    # Cart acceleration (no control force applied)
    temp = (
        Pendulum_Mass * Pole_Length * (theta_dot**2) * sin_theta
        + Pendulum_Mass * Gravity * sin_theta * cos_theta
    )
    x_ddot = temp / (total_mass - Pendulum_Mass * (cos_theta**2))

    # Pendulum angular acceleration
    theta_ddot = (Gravity * sin_theta - x_ddot * cos_theta) / Pole_Length

    return [x_dot, x_ddot, theta_dot, theta_ddot]


# --- Main Loop ---
running = True
FPS = 30

while running:
    # 1. Delta time calculation (seconds)
    dt = clock.tick(FPS) / 1000.0

    # Handle quit events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # 2. Step physics forward by dt using solve_ivp
    if dt > 0:
        sol = solve_ivp(
            fun=pendulum_dynamics,
            t_span=(0, dt),
            y0=state,
            method="RK45",
        )
        state = sol.y[:, -1]  # Extract state at end of dt step

    # Unpack current state
    x, _, theta, _ = state

    # 3. Convert physics coordinates to screen pixels
    cart_pixel_x = int(ORIGIN_X + x * SCALE)
    cart_pixel_y = ORIGIN_Y

    # Pendulum tip positions (cos for Y, sin for X)
    pole_pixel_x = int(cart_pixel_x + (Pole_Length * SCALE) * np.sin(theta))
    pole_pixel_y = int(cart_pixel_y - (Pole_Length * SCALE) * np.cos(theta))

    # 4. Render
    screen.fill(pygame.Color('black'))  # Clear background (black)

    # Draw track
    pygame.draw.line(
        screen, (pygame.Color('white')), (0, ORIGIN_Y + 15), (WIDTH, ORIGIN_Y + 15), 3
    )

    # Draw cart (centered on cart_pixel_x)
    cart_rect = pygame.Rect(0, 0, 80, 30)
    cart_rect.center = (cart_pixel_x, cart_pixel_y)
    pygame.draw.rect(screen, (pygame.Color('blue')), cart_rect)

    # Draw pendulum pole
    pygame.draw.line(
        screen,
        (pygame.Color('red')),

        (cart_pixel_x, cart_pixel_y),
        (pole_pixel_x, pole_pixel_y),
        6,
    )

    # Draw pendulum bob mass
    pygame.draw.circle(
        screen, (pygame.Color('red')), (pole_pixel_x, pole_pixel_y), 12
    )

    pygame.display.flip()

pygame.quit()
sys.exit()