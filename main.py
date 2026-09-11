import sys
import numpy as np
import pygame
import config
from dynamics import step_physics
from controller import compute_control_force

def main():
    pygame.init()
    screen = pygame.display.set_mode((config.WIDTH, config.HEIGHT))
    pygame.display.set_caption("Real-Time Inverted Pendulum Simulation")
    clock = pygame.time.Clock()

    state = list(config.INITIAL_STATE)
    running = True

    while running:
        dt = clock.tick(config.FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # 1. Compute control force
        force_u = compute_control_force(state)

        # 2. Integrate physics forward
        state = step_physics(state, dt, force_u)

        x, _, theta, _ = state

        # 3. Calculate screen locations
        cart_pixel_x = int(config.ORIGIN_X + x * config.SCALE)
        cart_pixel_y = config.ORIGIN_Y

        pole_pixel_x = int(cart_pixel_x + (config.POLE_LENGTH * config.SCALE) * np.sin(theta))
        pole_pixel_y = int(cart_pixel_y - (config.POLE_LENGTH * config.SCALE) * np.cos(theta))

        # 4. Drawing calls
        screen.fill(config.COLOR_BG)

        # Track
        pygame.draw.line(
            screen, config.COLOR_TRACK, 
            (0, config.ORIGIN_Y + 15), (config.WIDTH, config.ORIGIN_Y + 15), 3
        )

        # Cart
        cart_rect = pygame.Rect(0, 0, 80, 30)
        cart_rect.center = (cart_pixel_x, cart_pixel_y)
        pygame.draw.rect(screen, config.COLOR_CART, cart_rect)

        # Pendulum
        pygame.draw.line(
            screen, config.COLOR_POLE,
            (cart_pixel_x, cart_pixel_y), (pole_pixel_x, pole_pixel_y), 6
        )
        pygame.draw.circle(
            screen, config.COLOR_POLE, (pole_pixel_x, pole_pixel_y), 12
        )

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()