from realsense import RealSense
import motors

import argparse
import timeit
import time
from threading import Thread, Lock
import numpy as np
import pygame

import sys
sys.path.append('/usr/local/lib')
import pyrealsense2 as rs

WIDTH = 640
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


def normalize(v):
    n = np.linalg.norm(v)
    if n == 0:
        return v
    else:
        return v / n


def get_direction(depth_frame: rs.depth_frame, pixels):
    point_origin = get_real_world_coord(depth_frame, pixels[0])
    accum = np.zeros(3)
    for p in pixels[1:]:
        accum += normalize(get_real_world_coord(depth_frame, p) - point_origin)
    return accum / (len(pixels) - 1)


def compute_normal_vector(depth_frame: rs.depth_frame, screenX: int,
                          screenY: int):
    # assert screenX >= 0
    # assert screenX < depth_vals.shape[0] - 1
    # assert screenY >= 0
    # assert screenY < depth_vals.shape[1] - 1

    k = 5

    minX = max(0, screenX - k)
    maxX = min(WIDTH / RESCALE - 1, screenX + k)
    minY = max(0, screenY - k)
    maxY = min(HEIGHT / RESCALE - 1, screenY + k)

    ur_to_ul = [(min(WIDTH / RESCALE - 1, minX + i), minY) for i in range(k)]
    bl_to_ul = [(minX, min(HEIGHT / RESCALE - 1, minY + i)) for i in range(k)]

    # v1 = get_direction([(minX, minY), (minX + 1, minY))
    # point_ur = get_real_world_coord(depth_frame, (maxX, minY))
    # point_bl = get_real_world_coord(depth_frame, (minX, maxY))
    v1 = get_direction(depth_frame, ur_to_ul)
    v2 = get_direction(depth_frame, bl_to_ul)

    cross[1] = 0
    cross = np.cross(v2, v1)
    if np.linalg.norm(cross) == 0:
        return np.zeros(3)
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


depth_frame_global = None
depth_frame_lock = Lock()

frame_info_global = {
    "final": None,
    "mouse_x": None,
    "mouse_y": None,
    "vec": None
}
frame_info_lock = Lock()


class ThreadCounter():
    def __init__(self):
        self.lock = Lock()
        self.count = 0

    def inc(self):
        with self.lock:
            self.count += 1

    def get(self):
        tmp = 0
        with self.lock:
            tmp = self.count
        return tmp


class FrameThread(Thread):
    def __init__(self, camera):
        super().__init__()
        self.camera = camera
        self.frame_counter = ThreadCounter()
        self.done = False

    def run(self):
        global depth_frame_global
        while not self.done:
            # time.sleep(0.03)
            frame = self.camera.get_depth_frame()
            if not frame:
                continue

            data = np.kron(
                np.asanyarray(frame.get_data()).T.astype(float),
                np.ones((ZOOM, ZOOM)))

            with depth_frame_lock:
                depth_frame_global = frame

            with frame_info_lock:
                frame_info_global["final"] = data

            self.frame_counter.inc()


class MonitorThread(Thread):
    def __init__(self, threads):
        super().__init__()
        self.threads = threads
        self.done = False

    def run(self):
        prev_counts = {
            k: self.threads[k].frame_counter.get()
            for k in self.threads
        }
        while not self.done:
            time.sleep(1)
            new_counts = {
                k: self.threads[k].frame_counter.get()
                for k in self.threads
            }
            print("FPS: ", end='')
            for k in self.threads:
                print(k, end=' ')
                print(new_counts[k] - prev_counts[k], end=' ')
            prev_counts = new_counts
            print()


class MotorThread(Thread):
    def __init__(self, motor_controller):
        super().__init__()
        self.motor_controller = motor_controller
        self.done = False
        self.frame_counter = ThreadCounter()

    def run(self):
        global depth_frame_global
        print("motor thread starting")
        while not self.done:
            time.sleep(0.001)
            with depth_frame_lock:
                depth_frame = depth_frame_global
                if depth_frame == None:
                    continue

                pos = self.motor_controller.position()
                mouse_x = int(
                    max(
                        min((pos[0] / 50.8 + 0.25) * WIDTH / RESCALE * ZOOM /
                            1.25, WIDTH / RESCALE * ZOOM - 1), 0))
                mouse_y = int(
                    max(
                        min((pos[1] / 50.8 + 0.08) * WIDTH / RESCALE * ZOOM /
                            1.25 - 10, HEIGHT / RESCALE * ZOOM - 1), 0))

                z = max(int(pos[2]) * 400, 1)

                mouse_real_world = rs.rs2_deproject_pixel_to_point(
                    depth_frame.profile.as_video_stream_profile().intrinsics,
                    [mouse_x // 2, mouse_y // 2], z)

                vec = compute_feedback_vector(depth_frame, mouse_real_world)

            with frame_info_lock:
                frame_info_global["mouse_x"] = mouse_x
                frame_info_global["mouse_y"] = mouse_y
                frame_info_global["vec"] = vec

            self.motor_controller.create_force(vec, pos)
            self.frame_counter.inc()


def mainloop(screen):
    for event in pygame.event.get():
        if event.type == pygame.QUIT or event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            pygame.quit()
            return False

    frame_data = None
    with frame_info_lock:
        if frame_info_global["final"] is not None:
            frame_data = np.copy(frame_info_global["final"])
        mouse_x = frame_info_global["mouse_x"]
        mouse_y = frame_info_global["mouse_y"]
        vec = np.copy(frame_info_global["vec"])

    screen.fill((255, 255, 255))
    if frame_data is not None:
        to_display = frame_data // 32
        screenarray = np.zeros((WIDTH // RESCALE * ZOOM,
                                HEIGHT // RESCALE * ZOOM, 3))
        screenarray[:, :, 0] = to_display
        screenarray[:, :, 1] = to_display
        screenarray[:, :, 2] = to_display
        depth_image = pygame.surfarray.make_surface(screenarray)
        screen.blit(depth_image, (0, 0))

    if mouse_x is not None and mouse_y is not None:
        pygame.draw.circle(screen, (0, 255, 0), (mouse_x, mouse_y), 5)
        if vec is not None:
            pygame.draw.line(screen, (255, 0, 0), (mouse_x, mouse_y),
                             (mouse_x + int(vec[0]), mouse_y + int(vec[1])))

    pygame.display.flip()
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-s", "--screen", help="add visual display", action="store_true")
    args = parser.parse_args()

    print("initializing camera")
    camera = RealSense()
    camera.start_stream()
    frame_thread = FrameThread(camera)

    print("initializing motors")
    motor_controller = motors.Motors()
    motor_controller.start()
    # motor_controller.set_position(np.array((10, 10, 10)))
    motor_thread = MotorThread(motor_controller)

    monitor_thread = MonitorThread({
        "Camera": frame_thread,
        "Motors": motor_thread
    })
    motor_thread.start()
    frame_thread.start()
    monitor_thread.start()

    if args.screen:
        print("initializing pygame")
        pygame.init()
        display = (WIDTH // RESCALE * ZOOM, HEIGHT // RESCALE * ZOOM)
        screen = pygame.display.set_mode(display, 0)

        while mainloop(screen):
            time.sleep(0.03)
    else:
        time.sleep(100000)
    frame_thread.done = True
    motor_thread.done = True
    monitor_thread.done = True
    frame_thread.join()
    motor_thread.join()
    monitor_thread.join()


if __name__ == "__main__":
    main()
