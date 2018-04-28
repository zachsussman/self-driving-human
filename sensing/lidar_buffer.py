import threading
from queue import Queue,Empty

class LidarBuffer:
    def __init__(self, lidar):
        self.ld = lidar
        self.samples = []
        self.collecting = False

    def collectLoop(self):
        hi = 0
        started = False
        while True:
            try:
                if self.collecting:
                    item = self.q.get(timeout=0.5)
                else:
                    item = self.q.get_nowait()
            except Empty:
                if not self.collecting:
                    break
                else:
                    continue
            if item['first_scan']:
                if len(self.samples) > hi:
                    self.samples = self.samples[:hi]
                hi = 0
                started = True
            if not started:
                continue
            idx = hi
            while idx > 0 and self.samples[idx-1]['angle'] >= item['angle'] and self.samples[idx-1]['angle'] - item['angle'] < 20:
                idx -= 1
            self.samples.insert(idx, item)
            while idx+1 < len(self.samples) and self.samples[idx+1]['angle'] <= item['angle'] and item['angle'] - self.samples[idx+1]['angle'] < 20:
                self.samples.pop(idx+1)
            while idx+1 < len(self.samples) and self.samples[idx+1]['angle'] > item['angle'] + 20:
                self.samples.pop(idx+1)
            hi += 1

    def start(self):
        self.q = Queue()
        self.ld.startScan(self.q)
        self.collect_thread = threading.Thread(target=self.collectLoop)
        self.collecting = True
        self.collect_thread.start()

    def stop(self):
        self.collecting = False
        self.ld.stopScan()
