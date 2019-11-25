import sys
sys.path.append('/usr/local/lib')
import pyrealsense2 as rs
import numpy as np

WIDTH = 1280
HEIGHT = 720
RESCALE = 4


class RealSense():
    def __init__(self):
        self.filters = [
            rs.decimation_filter(RESCALE),
            rs.disparity_transform(True),
            rs.hole_filling_filter(1),
            rs.spatial_filter(0.5, 8, 2, 2),
            rs.temporal_filter(0.5, 20, 1),
            rs.disparity_transform(False)
        ]

    def start_stream(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
        self.pipeline.start(config)

    def get_depth_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None

        final_frame = depth_frame
        for f in self.filters:
            final_frame = f.process(final_frame)
        return final_frame

    def get_depth_vals(self):
        frame = self.get_depth_frame()
        if not frame:
            return None
        depth_vals = np.asanyarray(frame.get_data()).T.astype(float)
        return depth_vals