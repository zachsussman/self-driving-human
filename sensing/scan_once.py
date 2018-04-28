import time
from lidar import Lidar
from queue import Queue

if __name__ == "__main__":
    ld = Lidar("/dev/tty.SLAB_USBtoUART")
    print(ld.getDeviceInfo())
    print(ld.getDeviceHealth())
    q = Queue()
    ld.startScan(q)
    start_time = time.time()
    mode = 0
    while True:
        result = q.get()
        if result['first_scan']:
            if mode == 0:
                mode = 1
            elif mode == 1:
                mode = 2
            elif mode == 2:
                ld.stopScan()
                print(time.time() - start_time)
                exit(0)
        elif mode == 2:
            print(str(result) + ",")
