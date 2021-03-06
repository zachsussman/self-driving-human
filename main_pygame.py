from motors.pid import Controller, Motor, convert_polys, polygon_intersect
from motors.drawing import SCREEN_SIZE, convert2d, unconvert
from sensing.objectdetect import detect_polygons
from sensing.lidar_mock import Lidar
from sensing.lidar_buffer import LidarBuffer
from lidar_to_points import lidar_to_points
import numpy as np
import pygame
import serial



ser = None
try:
    ser = serial.Serial('/dev/tty.usbmodem00000001')
    print("connected to serial")
except:
    pass


outline = [(-3000, 0), (-2000, 1000), (-1000, 1500), (-500, 2000), (0, 1000), (1000, 3000), (2000, 2000), (2500, 200), (3000, 0)]

motor1 = Motor(np.array((-3000, 0, 0)), 0)
motor2 = Motor(np.array((-3000, 5050, 0)), 0)
motor3 = Motor(np.array((3000, 0, 0)), 0)
con = Controller([motor1, motor2, motor3], ser)

ld = Lidar("/dev/tty.SLAB_USBtoUART")
lb = LidarBuffer(ld)
lb.start()

samples = lb.samples[:]
print(samples)

con.set_position(np.array((0, 200, 1000)))



def end():
    pygame.quit()
    lb.stop()
    quit()


def main():
    pygame.init()
    display = (SCREEN_SIZE, SCREEN_SIZE)
    screen = pygame.display.set_mode(display, 0)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                end()

        # con.update_serial()

        mouse_pos = unconvert(pygame.mouse.get_pos())
        con.set_position(np.array((mouse_pos[0], 200, mouse_pos[1])))
        screen.fill((255, 255, 255))

        samples = lb.samples[:]
        # print(samples)
        # end()
        outline = lidar_to_points(samples)
        # polygons = convert_polys(detect_polygons(samples), 0.05)
        con.set_outline(outline)
        con.move_from_outline()

        # for poly in polygons:
        p = [convert2d(point) for point in outline]
            # color = (0, 128, 0) if polygon_intersect(p, mouse_pos) else (0, 0, 128)
        if len(p) > 1: pygame.draw.aalines(screen, (0, 0, 128), False, p)
        # pygame.draw.lines(screen, (0, 128, 0), False, [convert2d(p) for p in outline])
        con.draw(screen)
        # print("frame")

        pygame.display.flip()
        pygame.time.wait(30)

if __name__ == "__main__":
    main()
