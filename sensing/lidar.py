import serial
import time
import threading
import math

cmdSyncByte = 0xA5
cmdGetHealth = 0x52
cmdGetDeviceInfo = 0x50
cmdReset = 0x40
cmdStop = 0x25
cmdStart = 0x20

respSyncByte1 = cmdSyncByte
respSyncByte2 = 0x5A

modeSingleResponse = 0
modeMultipleResponse = 1
typeMeasurement = 0x81
typeDeviceInfo = 0x4
typeDeviceHealth = 0x6

def byte(b):
    return bytes([b])

class Lidar:
    def __init__(self, port):
        self.ser = serial.Serial(port=port,baudrate=115200)
        self.scan_thread = None
        self.scanning = False

    def sendCommand(self, cmd):
        self.ser.write(bytes([cmdSyncByte, cmd]))

    def waitForSyncBytes(self):
        prev_b = None
        while True:
            b = self.ser.read()
            if ord(b) == respSyncByte2 and ord(prev_b) == respSyncByte1:
                return
            prev_b = b

    def readBytes(self, n):
        bs = b""
        i = 0
        while i < n:
            b = self.ser.read()
            bs += b
            i += 1
        return bs

    def getResponseDescriptor(self):
        self.waitForSyncBytes()
        info = self.readBytes(5)
        return {
            "length": info[0] | (info[1] << 8) | (info[2] << 16) | ((info[3] & 0x3F) << 24),
            "mode": (info[3] & 0xC0) >> 6,
            "type": info[4],
        }

    def getDeviceInfo(self):
        self.sendCommand(cmdGetDeviceInfo)
        desc = self.getResponseDescriptor()
        assert(desc['length'] == 20)
        assert(desc['mode'] == modeSingleResponse)
        assert(desc['type'] == typeDeviceInfo)
        info = self.readBytes(desc['length'])
        return {
            "model": info[0],
            "firmwareMinor": info[1],
            "firmwareMajor": info[2],
            "hardware": info[3],
            "serialNumber": info[4:]
        }

    def getDeviceHealth(self):
        self.sendCommand(cmdGetHealth)
        desc = self.getResponseDescriptor()
        assert(desc['length'] == 3)
        assert(desc['mode'] == modeSingleResponse)
        assert(desc['type'] == typeDeviceHealth)
        info = self.readBytes(desc['length'])
        return {
            "status": info[0],
            "errorCode": info[1] | (info[2] << 8)
        }

    def scanLoop(self, queue):
        while self.scan_thread != None:
            info = self.readBytes(5)
            assert(info[0] & 0x1 != ((info[0] & 0x2) >> 1))
            assert(info[1] & 0x1 != 0)
            angleFixed = ((info[1] & 0xFE) >> 1) | (info[2] << 7);
            distanceFixed = info[3] | (info[4] << 8);
            queue.put({
                "first_scan": bool(info[0] & 0x1),
                "quality": (info[0] & 0xFC) >> 2,
                "angle": float(angleFixed) / 64.0,      # degrees
                "distance": float(distanceFixed) / 4.0  # millimeters
            })
        self.scanning = False

    def stopScan(self):
        self.scan_thread = None
        while self.scanning:
            pass
        self.sendCommand(cmdStop)
        time.sleep(1.0/1000)
        self.ser.reset_input_buffer()

    def startScan(self, queue):
        self.stopScan()
        self.sendCommand(cmdStart)
        desc = self.getResponseDescriptor()
        assert(desc['length'] == 5)
        assert(desc['mode'] == modeMultipleResponse)
        assert(desc['type'] == typeMeasurement)
        self.scan_thread = threading.Thread(target=self.scanLoop, args=(queue,))
        self.scanning = True
        self.scan_thread.start()
