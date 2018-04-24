import math
from lidar_mock import Lidar
from lidar_buffer import LidarBuffer
from queue import Queue
from tkinter import *
from objectdetect import *

scale = 0.1

def create_point(canvas, x, y):
    canvas.create_oval(x-1, y-1, x+1, y+1, fill="black", tags='point')

if __name__ == "__main__":
    try:
        tk = Tk()
        canvas = Canvas(tk, width=800, height=800)
        canvas.pack()
        canvas.create_oval(400-5, 400-5, 400+5, 400+5, fill="blue", width=0, tags="origin")
        ld = Lidar("/dev/tty.SLAB_USBtoUART")
        print(ld.getDeviceInfo())
        print(ld.getDeviceHealth())
        lb = LidarBuffer(ld)

        def on_close():
            global tk
            lb.stop()
            tk.destroy()
            tk = None
        tk.protocol("WM_DELETE_WINDOW", on_close)

        lb.start()

        while True:
            canvas.delete('point')
            samples = lb.samples[:]
            polygons = detect_polygons(samples)
            for polygon in polygons:
                for i in range(len(polygon)):
                    curr = polygon[i]
                    next = polygon[(i+1) % len(polygon)]
                    x1 = 400 + curr['x'] * scale
                    y1 = 400 - curr['y'] * scale
                    x2 = 400 + next['x'] * scale
                    y2 = 400 - next['y'] * scale
                    canvas.create_line(x1, y1, x2, y2, fill="gray", width=2, tags='point')
                    create_point(canvas, x1, y1)
            tk.update_idletasks()
            tk.update()
            if not tk:
                exit(0)
    except KeyboardInterrupt:
        on_close()
