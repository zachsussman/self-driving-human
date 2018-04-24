import threading
from queue import Queue,Empty

class LidarBuffer:
    def __init__(self, lidar):
        self.ld = lidar
        self.samples = []
        self.collecting = False

    def collectLoop(self):
        idx = 0
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
                if len(self.samples) > idx:
                    self.samples = self.samples[:idx]
                idx = 0
                started = True
            if not started:
                continue
            if idx < len(self.samples):
                self.samples[idx] = item
            else:
                self.samples.append(item)
            idx += 1

    def start(self):
        self.q = Queue()
        self.ld.startScan(self.q)
        self.collect_thread = threading.Thread(target=self.collectLoop)
        self.collecting = True
        self.collect_thread.start()

    def stop(self):
        self.collecting = False
        self.ld.stopScan()
