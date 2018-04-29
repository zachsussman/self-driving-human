import math

RAD_PER_DEGREE = math.pi * 2 / 360

def lidar_to_points(samples):
    angular_points = [(s['distance'] if s['quality'] != 0 else 100000, s['angle']) for s in samples]
    angular_points.sort(key=lambda p: ((p[1] - 270) % 360))
    rect = [(p[0] * math.sin(p[1] * RAD_PER_DEGREE), p[0] * math.cos(p[1] * RAD_PER_DEGREE)) for p in angular_points]
    rect = [r for r in rect if r[1] > 0]
    return rect