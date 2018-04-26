from motors.controller import Controller, Motor, convert_polys, polygon_intersect
from sensing.objectdetect import detect_polygons
from sensing.lidar_mock import Lidar
from sensing.lidar_buffer import LidarBuffer
import numpy as np
import pygame
import serial


COUNTS_PER_INCH = 372.14285714285717

motor1 = Motor(np.array((0, 0, 0)), 1)
motor2 = Motor(np.array((500, 20, 0)), 1)
motor3 = Motor(np.array((450, 500, 0)), 2)
con = Controller([motor1, motor2, motor3])

ld = Lidar("/dev/tty.SLAB_USBtoUART")
lb = LidarBuffer(ld)
lb.start()

ser = None
try:
    ser = serial.Serial('/dev/ttyUSB0')
except:
    pass

print("Hi")

def end():
    pygame.quit()
    lb.stop()
    quit()


def main():
    pygame.init()
    display = (600, 600)
    screen = pygame.display.set_mode(display, 0)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                end()

        while ser and ser.in_waiting > 0:
            data = ser.read(5)
            motor_index = {'A': 0, 'B': 1, 'C': 2}
            motor_to_update = con.motors[motor_index[data[0]]]
            new_encoder = int(data[1:], 16)
            motor_to_update.line = new_encoder / COUNTS_PER_INCH


        mouse_pos = pygame.mouse.get_pos()
        con.set_position(np.array((mouse_pos[0], 200, mouse_pos[1])))
        screen.fill((255, 255, 255))

        samples = lb.samples[:]
        polygons = convert_polys(detect_polygons(samples), 0.1)
        con.set_polygons(polygons)
        con.set_correct_force()

        for poly in polygons:
            color = (0, 128, 0) if polygon_intersect(poly, mouse_pos) else (0, 0, 128)
            pygame.draw.polygon(screen, color, poly)
        con.draw(screen)

        pygame.display.flip()
        pygame.time.wait(30)

if __name__ == "__main__":
    main()