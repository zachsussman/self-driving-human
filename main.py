from motors.controller import Controller, Motor, convert_polys, polygon_intersect
from motors.drawing import SCREEN_SIZE, convert2d, unconvert
from sensing.objectdetect import detect_polygons
from sensing.lidar import Lidar
from sensing.lidar_buffer import LidarBuffer
import numpy as np
import serial



ser = None
try:
    ser = serial.Serial('/dev/tty.usbmodem00000001')
    print("connected to serial")
except:
    pass


outline = [(-3000, 0), (-2000, 1000), (-1000, 1500), (-500, 2000), (0, 1000), (1000, 3000), (2000, 2000), (2500, 200), (3000, 0)]

motor1 = Motor(np.array((-3000, 0, 0)), 1)
motor2 = Motor(np.array((3000, 0, 0)), 1)
motor3 = Motor(np.array((-3000, 5000, 0)), 2)
con = Controller([motor1, motor2, motor3], ser)

ld = Lidar("/dev/tty.SLAB_USBtoUART")
lb = LidarBuffer(ld)
lb.start()

samples = lb.samples[:]
print(samples)

con.set_position(np.array((0, 200, 1000)))



def end():
    lb.stop()
    quit()


def main():
    while True:
        con.update_serial()

        con.set_outline(outline)

        samples = lb.samples[:]
        polygons = convert_polys(detect_polygons(samples), 0.05)
        if len(polygons) == 0: polygons = [[]]
        con.set_outline(polygons[0])
        con.move_from_outline()

if __name__ == "__main__":
    main()
