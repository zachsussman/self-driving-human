from realsense import RealSense
import motors

import timeit

import numpy as np
import pygame

import sys
sys.path.append('/usr/local/lib')
import pyrealsense2 as rs

WIDTH = 848
HEIGHT = 480
RESCALE = 4
ZOOM = 2


def get_real_world_coord(depth_frame: rs.depth_frame, pos: (int, int)):
    x, y = pos
    depth_value = depth_frame.as_depth_frame().get_distance(int(x),
                                                            int(y)) * 1000
    return np.array(
        rs.rs2_deproject_pixel_to_point(
            depth_frame.profile.as_video_stream_profile().intrinsics, [x, y],
            depth_value))


def compute_normal_vector(depth_frame: rs.depth_frame, screenX: int,
                          screenY: int):
    # assert screenX >= 0
    # assert screenX < depth_vals.shape[0] - 1
    # assert screenY >= 0
    # assert screenY < depth_vals.shape[1] - 1

    point_ul = get_real_world_coord(depth_frame, (screenX, screenY))
    point_ur = get_real_world_coord(depth_frame, (screenX + 1, screenY))
    point_bl = get_real_world_coord(depth_frame, (screenX, screenY + 1))
    v1 = point_ur - point_ul
    v2 = point_bl - point_ul

    cross = np.cross(v2, v1)
    return cross / np.linalg.norm(cross)


def compute_feedback_vector(depth_frame: rs.depth_frame, pos3d: (float, float,
                                                                 float)):
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    x, y = rs.rs2_project_point_to_pixel(depth_intrin, pos3d)
    plane_depth = depth_frame.as_depth_frame().get_distance(int(x),
                                                            int(y)) * 1000

    if plane_depth > pos3d[2]:
        return np.array((0, 0, 0))

    dist_beyond = pos3d[2] - plane_depth
    return dist_beyond * compute_normal_vector(depth_frame, x, y)


def main():
    print("initializing camera")
    camera = RealSense()
    camera.start_stream()

    print("initializing motors")
    motor_controller = motors.Motors()
    motor_controller.start()

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

        # (mouse_x, mouse_y) = pygame.mouse.get_pos()
        pos = motor_controller.position()
        mouse_x = int(
            max(
                min(pos[0] / 50.8 * WIDTH / RESCALE * ZOOM,
                    WIDTH / RESCALE * ZOOM - 1), 0))
        mouse_y = int(
            max(
                min(pos[1] / 50.8 * WIDTH / RESCALE * ZOOM - 10,
                    HEIGHT / RESCALE * ZOOM - 1), 0))

        z = max(int(pos[2]) * 150, 1)

        mouse_real_world = rs.rs2_deproject_pixel_to_point(
            depth_frame.profile.as_video_stream_profile().intrinsics,
            [mouse_x // 2, mouse_y // 2], z)

        to_display = final // 32
        screenarray = np.zeros((WIDTH // RESCALE * ZOOM,
                                HEIGHT // RESCALE * ZOOM, 3))
        screenarray[:, :, 0] = to_display
        screenarray[:, :, 1] = to_display
        screenarray[:, :, 2] = to_display

        depth_image = pygame.surfarray.make_surface(screenarray)

        vec = compute_feedback_vector(depth_frame, mouse_real_world)
        if np.isnan(vec[0]):
            vec = np.array((0, 0, 0))

        motor_controller.create_force(vec)

        screen.fill((255, 255, 255))
        screen.blit(depth_image, (0, 0))
        pygame.draw.circle(screen, (0, 255, 0), (mouse_x, mouse_y), 5)
        pygame.draw.line(screen, (255, 0, 0), (mouse_x, mouse_y),
                         (mouse_x + int(vec[0]), mouse_y + int(vec[1])))
        pygame.display.flip()
        pygame.time.wait(30)


if __name__ == "__main__":
    main()
