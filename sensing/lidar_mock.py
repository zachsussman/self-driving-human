import time
import threading

data = [
{'first_scan': True, 'quality': 30, 'angle': 351.390625, 'distance': 2246.75},
{'first_scan': False, 'quality': 29, 'angle': 355.234375, 'distance': 2138.0},
{'first_scan': False, 'quality': 19, 'angle': 359.015625, 'distance': 2347.75},
{'first_scan': False, 'quality': 25, 'angle': 2.828125, 'distance': 2632.0},
{'first_scan': False, 'quality': 18, 'angle': 6.59375, 'distance': 3000.0},
{'first_scan': False, 'quality': 12, 'angle': 10.4375, 'distance': 3505.5},
{'first_scan': False, 'quality': 9, 'angle': 14.203125, 'distance': 4247.0},
{'first_scan': False, 'quality': 0, 'angle': 25.6875, 'distance': 0.0},
{'first_scan': False, 'quality': 9, 'angle': 21.78125, 'distance': 6954.75},
{'first_scan': False, 'quality': 0, 'angle': 33.390625, 'distance': 0.0},
{'first_scan': False, 'quality': 0, 'angle': 37.21875, 'distance': 0.0},
{'first_scan': False, 'quality': 0, 'angle': 41.078125, 'distance': 0.0},
{'first_scan': False, 'quality': 0, 'angle': 44.96875, 'distance': 0.0},
{'first_scan': False, 'quality': 10, 'angle': 41.546875, 'distance': 2089.75},
{'first_scan': False, 'quality': 29, 'angle': 45.5, 'distance': 1743.75},
{'first_scan': False, 'quality': 31, 'angle': 49.40625, 'distance': 1775.0},
{'first_scan': False, 'quality': 31, 'angle': 53.1875, 'distance': 1814.5},
{'first_scan': False, 'quality': 30, 'angle': 57.046875, 'distance': 1864.75},
{'first_scan': False, 'quality': 28, 'angle': 60.890625, 'distance': 1728.25},
{'first_scan': False, 'quality': 29, 'angle': 64.859375, 'distance': 1532.75},
{'first_scan': False, 'quality': 29, 'angle': 68.75, 'distance': 1480.75},
{'first_scan': False, 'quality': 29, 'angle': 72.546875, 'distance': 1564.0},
{'first_scan': False, 'quality': 28, 'angle': 76.328125, 'distance': 1659.75},
{'first_scan': False, 'quality': 32, 'angle': 80.0, 'distance': 2061.25},
{'first_scan': False, 'quality': 23, 'angle': 84.0, 'distance': 1961.5},
{'first_scan': False, 'quality': 17, 'angle': 87.796875, 'distance': 2151.5},
{'first_scan': False, 'quality': 14, 'angle': 91.5625, 'distance': 2401.75},
{'first_scan': False, 'quality': 0, 'angle': 102.8125, 'distance': 0.0},
{'first_scan': False, 'quality': 10, 'angle': 99.015625, 'distance': 4411.5},
{'first_scan': False, 'quality': 9, 'angle': 102.75, 'distance': 6999.5},
{'first_scan': False, 'quality': 10, 'angle': 106.53125, 'distance': 6893.25},
{'first_scan': False, 'quality': 12, 'angle': 110.359375, 'distance': 6752.5},
{'first_scan': False, 'quality': 0, 'angle': 121.984375, 'distance': 0.0},
{'first_scan': False, 'quality': 0, 'angle': 125.84375, 'distance': 0.0},
{'first_scan': False, 'quality': 0, 'angle': 129.671875, 'distance': 0.0},
{'first_scan': False, 'quality': 0, 'angle': 133.53125, 'distance': 0.0},
{'first_scan': False, 'quality': 0, 'angle': 137.359375, 'distance': 0.0},
{'first_scan': False, 'quality': 14, 'angle': 134.09375, 'distance': 1718.25},
{'first_scan': False, 'quality': 22, 'angle': 138.234375, 'distance': 1276.25},
{'first_scan': False, 'quality': 23, 'angle': 142.328125, 'distance': 1016.5},
{'first_scan': False, 'quality': 28, 'angle': 146.484375, 'distance': 860.5},
{'first_scan': False, 'quality': 25, 'angle': 150.234375, 'distance': 888.5},
{'first_scan': False, 'quality': 28, 'angle': 154.015625, 'distance': 924.5},
{'first_scan': False, 'quality': 28, 'angle': 157.875, 'distance': 965.5},
{'first_scan': False, 'quality': 29, 'angle': 161.65625, 'distance': 1015.0},
{'first_scan': False, 'quality': 28, 'angle': 165.4375, 'distance': 1077.25},
{'first_scan': False, 'quality': 33, 'angle': 169.21875, 'distance': 1151.25},
{'first_scan': False, 'quality': 31, 'angle': 173.015625, 'distance': 1240.75},
{'first_scan': False, 'quality': 30, 'angle': 176.78125, 'distance': 1351.75},
{'first_scan': False, 'quality': 16, 'angle': 183.453125, 'distance': 357.5},
{'first_scan': False, 'quality': 19, 'angle': 187.3125, 'distance': 357.25},
{'first_scan': False, 'quality': 11, 'angle': 191.125, 'distance': 386.75},
{'first_scan': False, 'quality': 0, 'angle': 199.03125, 'distance': 0.0},
{'first_scan': False, 'quality': 28, 'angle': 198.8125, 'distance': 392.25},
{'first_scan': False, 'quality': 28, 'angle': 202.390625, 'distance': 389.0},
{'first_scan': False, 'quality': 29, 'angle': 206.625, 'distance': 384.5},
{'first_scan': False, 'quality': 28, 'angle': 210.09375, 'distance': 395.5},
{'first_scan': False, 'quality': 29, 'angle': 214.046875, 'distance': 395.75},
{'first_scan': False, 'quality': 31, 'angle': 218.234375, 'distance': 394.0},
{'first_scan': False, 'quality': 20, 'angle': 221.828125, 'distance': 411.0},
{'first_scan': False, 'quality': 26, 'angle': 225.453125, 'distance': 427.5},
{'first_scan': False, 'quality': 26, 'angle': 229.1875, 'distance': 430.25},
{'first_scan': False, 'quality': 17, 'angle': 232.78125, 'distance': 472.0},
{'first_scan': False, 'quality': 14, 'angle': 236.875, 'distance': 449.75},
{'first_scan': False, 'quality': 22, 'angle': 240.828125, 'distance': 424.75},
{'first_scan': False, 'quality': 0, 'angle': 249.328125, 'distance': 0.0},
{'first_scan': False, 'quality': 0, 'angle': 253.1875, 'distance': 0.0},
{'first_scan': False, 'quality': 25, 'angle': 257.078125, 'distance': 0.0},
{'first_scan': False, 'quality': 26, 'angle': 253.46875, 'distance': 2195.0},
{'first_scan': False, 'quality': 21, 'angle': 257.328125, 'distance': 2165.25},
{'first_scan': False, 'quality': 31, 'angle': 261.234375, 'distance': 2036.25},
{'first_scan': False, 'quality': 9, 'angle': 264.640625, 'distance': 5498.5},
{'first_scan': False, 'quality': 0, 'angle': 276.21875, 'distance': 0.0},
{'first_scan': False, 'quality': 32, 'angle': 272.90625, 'distance': 1756.5},
{'first_scan': False, 'quality': 30, 'angle': 276.765625, 'distance': 1695.25},
{'first_scan': False, 'quality': 29, 'angle': 280.71875, 'distance': 1538.5},
{'first_scan': False, 'quality': 27, 'angle': 284.609375, 'distance': 1507.75},
{'first_scan': False, 'quality': 28, 'angle': 288.515625, 'distance': 1471.25},
{'first_scan': False, 'quality': 12, 'angle': 292.125, 'distance': 1947.5},
{'first_scan': False, 'quality': 0, 'angle': 303.21875, 'distance': 0.0},
{'first_scan': False, 'quality': 23, 'angle': 299.265625, 'distance': 4441.5},
{'first_scan': False, 'quality': 26, 'angle': 303.109375, 'distance': 4428.25},
{'first_scan': False, 'quality': 23, 'angle': 306.9375, 'distance': 4448.0},
{'first_scan': False, 'quality': 20, 'angle': 310.78125, 'distance': 4481.25},
{'first_scan': False, 'quality': 20, 'angle': 314.59375, 'distance': 4566.0},
{'first_scan': False, 'quality': 20, 'angle': 318.40625, 'distance': 4672.5},
{'first_scan': False, 'quality': 20, 'angle': 346.28125, 'distance': 4747.25},
{'first_scan': False, 'quality': 0, 'angle': 333.8125, 'distance': 0.0},
{'first_scan': False, 'quality': 16, 'angle': 330.0625, 'distance': 3694.75},
{'first_scan': False, 'quality': 11, 'angle': 333.953125, 'distance': 3270.5},
{'first_scan': False, 'quality': 0, 'angle': 345.328125, 'distance': 0.0},
{'first_scan': False, 'quality': 21, 'angle': 341.6875, 'distance': 2838.0},
{'first_scan': False, 'quality': 29, 'angle': 345.578125, 'distance': 2617.25}
]
idx = 0

class Lidar:
    def __init__(self, port):
        self.scan_thread = None
        self.scanning = False

    def getDeviceInfo(self):
        return {
            'model': 40,
            'firmwareMinor': 20,
            'firmwareMajor': 1,
            'hardware': 2,
            'serialNumber': b'\xb8\xd2\xb5\xc1\xe8\x83\x9e\xf2\xc0\xe6\x9e\xf7`e\x1e\x05'
        }

    def getDeviceHealth(self):
        return {'status': 0, 'errorCode': 0}

    def scanLoop(self, queue):
        global idx
        while self.scan_thread != None:
            queue.put(data[idx])
            idx = (idx + 1) % len(data)
            time.sleep(0.006)
        self.scanning = False

    def stopScan(self):
        self.scan_thread = None
        while self.scanning:
            pass
        time.sleep(1.0/1000)

    def startScan(self, queue):
        self.stopScan()
        self.scan_thread = threading.Thread(target=self.scanLoop, args=(queue,))
        self.scanning = True
        self.scan_thread.start()
