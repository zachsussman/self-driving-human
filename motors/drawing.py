import math

SCREEN_SIZE = 600
LIDAR_DISTANCE = 3000

CVT = SCREEN_SIZE / LIDAR_DISTANCE / 2


def convert3d(a):
    if math.isnan(a[0]) or math.isnan(a[2]): return (0, 0)
    return (int((a[0] + LIDAR_DISTANCE) * CVT), int(SCREEN_SIZE - a[2]*CVT))

def convert2d(a):
    if math.isnan(a[0]) or math.isnan(a[1]): return (0, 0)
    return (int((a[0] + LIDAR_DISTANCE) * CVT), int(SCREEN_SIZE - a[1]*CVT))

def unconvert(a):
    return (a[0]/CVT - LIDAR_DISTANCE, (SCREEN_SIZE - a[1]) / CVT)