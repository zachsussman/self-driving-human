SCREEN_SIZE = 600
LIDAR_DISTANCE = 3000

CVT = SCREEN_SIZE / LIDAR_DISTANCE / 2


def convert3d(a):
    return ((a[0] + LIDAR_DISTANCE) * CVT, SCREEN_SIZE - a[2]*CVT)

def convert2d(a):
    return ((a[0] + LIDAR_DISTANCE) * CVT, SCREEN_SIZE - a[1]*CVT)

def unconvert(a):
    return (a[0]/CVT - LIDAR_DISTANCE, (SCREEN_SIZE - a[1]) / CVT)