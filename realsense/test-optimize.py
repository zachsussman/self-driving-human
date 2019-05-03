import pygame
import motors
import numpy as np
import scipy.optimize
import random

M1_LOC = np.array([0, 0, 0])
M2_LOC = np.array([200, 0, 0])
M3_LOC = np.array([200, 200, 0])
MOTOR_LOCS = [M1_LOC, M2_LOC, M3_LOC]

MIDDLE_LOC = sum(MOTOR_LOCS) / len(MOTOR_LOCS)
MIDDLE_LOC[2] = 30


def dist(va, vb):
    return ((va[0] - vb[0])**2 + (va[1] - vb[1])**2 + (va[2] - vb[2])**2)**0.5


avg = None


def go_through_tri(x, y):
    global avg
    p = np.array((x, y, 100))
    encs = np.array([dist(p, l) for l in MOTOR_LOCS])
    q = trilaterate(encs)
    print(int(q[2]))
    return (int(q[0]), int(q[1]))


def trilaterate(encs):
    def mse(x, locations, distances):
        ret = 0.0
        for l, d in zip(locations, distances):
            calc = np.sqrt((x[0] - l[0])**2 + (x[1] - l[1])**2 +
                           (x[2] - l[2])**2)
            ret += (calc - d)**2
        # print("distance to", x, "is", ret)
        return ret

    return scipy.optimize.minimize(
        mse, MIDDLE_LOC, args=([M1_LOC, M2_LOC, M3_LOC], encs)).x


def main():
    print("initializing camera")

    print("initializing motors")
    # motor_controller = motors.Motors()
    # motor_controller.start()

    print("initializing pygame")
    pygame.init()
    display = (500, 500)
    screen = pygame.display.set_mode(display, 0)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.quit()
                return

        # pos = motor_controller.position()
        # print("Current pos: ", int(pos[0]), int(pos[1]), int(pos[2]))
        # mouse_x = int(
        #     max(
        #         min(pos[0] / 50.8 * WIDTH / RESCALE * ZOOM,
        #             WIDTH / RESCALE * ZOOM - 1), 0))
        # mouse_y = int(
        #     max(
        #         min(pos[1] / 50.8 * WIDTH / RESCALE * ZOOM - 10,
        #             HEIGHT / RESCALE * ZOOM - 1), 0))

        (mouse_x, mouse_y) = pygame.mouse.get_pos()
        mouse_x, mouse_y = go_through_tri(mouse_x, mouse_y)

        # z = max(int(pos[2]) * 100, 1)

        screen.fill((0, 0, 0))
        # screen.blit(depth_image, (0, 0))
        pygame.draw.circle(screen, (0, 255, 0), (mouse_x, mouse_y), 5)
        pygame.display.flip()
        pygame.time.wait(30)


main()