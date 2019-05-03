from realsense import RealSense

import numpy as np
import pygame

import sys
sys.path.append('/usr/local/lib')
import pyrealsense2 as rs

WIDTH = 848
HEIGHT = 480
RESCALE = 4
ZOOM = 2


def main():
    print("initializing camera")
    camera = RealSense()
    camera.start_stream()

    print("initializing pygame")
    pygame.init()
    display = (WIDTH // RESCALE * ZOOM, HEIGHT // RESCALE * ZOOM)
    screen = pygame.display.set_mode(display, 0)

    z = 2000

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                pygame.quit()
                return
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    z += 100
                elif event.key == pygame.K_w:
                    z -= 100

        depth_frame = camera.get_depth_frame()
        if not depth_frame:
            continue

        final = np.kron(
            np.asanyarray(depth_frame.get_data()).T.astype(float),
            np.ones((ZOOM, ZOOM)))

        to_display = final // 32
        screenarray = np.zeros((WIDTH // RESCALE * ZOOM,
                                HEIGHT // RESCALE * ZOOM, 3))
        screenarray[:, :, 0] = to_display
        screenarray[:, :, 1] = to_display
        screenarray[:, :, 2] = to_display

        depth_image = pygame.surfarray.make_surface(screenarray)

        screen.fill((255, 255, 255))
        screen.blit(depth_image, (0, 0))
        pygame.display.flip()
        pygame.time.wait(30)


if __name__ == "__main__":
    main()