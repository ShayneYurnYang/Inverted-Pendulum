import sys
from collections import deque

import numpy as np
import pygame

import config
from controller import compute_control_force
from dynamics import step_physics


def main():
    pygame.init()
    screen = pygame.display.set_mode((config.WIDTH, config.HEIGHT))
    pygame.display.set_caption("Real-Time Inverted Pendulum Simulation")
    clock = pygame.time.Clock()

    state = list(config.INITIAL_STATE)

    # Keep a history for delayed measurements.
    delay_frames = int(getattr(config, "LATENCY_SEC", 0.0) * config.FPS)
    buffer_len = max(1, delay_frames)
    state_history = deque([list(state)] * buffer_len, maxlen=buffer_len)

    running = True

    while running:
        dt = clock.tick(config.FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_LEFT:
                    state[3] -= 1.0
                elif event.key == pygame.K_RIGHT:
                    state[3] += 1.0

                elif event.key == pygame.K_w or event.key == pygame.K_UP:
                    state[5] -= 1.5
                elif event.key == pygame.K_s or event.key == pygame.K_DOWN:
                    state[5] += 1.5

                elif event.key == pygame.K_a:
                    state[1] -= 0.5
                elif event.key == pygame.K_d:
                    state[1] += 0.5

                elif event.key == pygame.K_r:
                    state = list(config.INITIAL_STATE)
                    state_history = deque([list(state)] * buffer_len, maxlen=buffer_len)
        state_history.append(list(state))

        # Apply control from the delayed state.
        delayed_state = state_history[0]
        force_u = compute_control_force(delayed_state)

        state = step_physics(state, dt, force_u)

        cart_pixel_x = int(config.ORIGIN_X + state[0] * config.SCALE)
        cart_pixel_y = config.ORIGIN_Y

        is_double = len(state) == 6

        if is_double:
            x, _, theta1, _, theta2, _ = state
            p1_x = int(cart_pixel_x + (config.POLE_LEN_1 * config.SCALE) * np.sin(theta1))
            p1_y = int(cart_pixel_y - (config.POLE_LEN_1 * config.SCALE) * np.cos(theta1))
            p2_x = int(p1_x + (config.POLE_LEN_2 * config.SCALE) * np.sin(theta2))
            p2_y = int(p1_y - (config.POLE_LEN_2 * config.SCALE) * np.cos(theta2))
        else:
            x, _, theta, _ = state
            p1_x = int(cart_pixel_x + (config.POLE_LENGTH * config.SCALE) * np.sin(theta))
            p1_y = int(cart_pixel_y - (config.POLE_LENGTH * config.SCALE) * np.cos(theta))

        screen.fill(config.COLOR_BG)

        pygame.draw.line(
            screen, config.COLOR_TRACK, 
            (0, config.ORIGIN_Y + 15), (config.WIDTH, config.ORIGIN_Y + 15), 3
        )

        cart_rect = pygame.Rect(0, 0, 80, 30)
        cart_rect.center = (cart_pixel_x, cart_pixel_y)
        pygame.draw.rect(screen, config.COLOR_CART, cart_rect)

        if is_double:
            pygame.draw.line(screen, getattr(config, "COLOR_POLE1", config.COLOR_POLE1), (cart_pixel_x, cart_pixel_y), (p1_x, p1_y), 6)
            pygame.draw.circle(screen, getattr(config, "COLOR_POLE1", config.COLOR_POLE1), (p1_x, p1_y), 8)
            pygame.draw.line(screen, getattr(config, "COLOR_POLE2", config.COLOR_POLE2), (p1_x, p1_y), (p2_x, p2_y), 4)
            pygame.draw.circle(screen, getattr(config, "COLOR_POLE2", config.COLOR_POLE2), (p2_x, p2_y), 6)
        else:
            pygame.draw.line(screen, getattr(config, "COLOR_POLE1", config.COLOR_POLE1), (cart_pixel_x, cart_pixel_y), (p1_x, p1_y), 6)
            pygame.draw.circle(screen, getattr(config, "COLOR_POLE1", config.COLOR_POLE1), (p1_x, p1_y), 12)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()