import pygame
import numpy as np
from motors.drawing import convert3d
import string
import math

def convert_polys(polys, scale):
    return [[(point['x'], point['y']) for point in poly] for poly in polys]

''' normalize: np.array -> np.array '''
def normalize(v):
    ''' Normalizes the vector v '''
    return v / np.linalg.norm(v)


''' vector_to_line: np.array[2] * np.array[2] * np.array[2] -> np.array[2] '''
def vector_to_line(A, B, P):
    ''' Returns the shortest vector from the point P to the line segment AB '''
    # https://gist.github.com/nim65s/5e9902cd67f094ce65b0
    if all(A == P) or all(B == P):
        # at one endpoint
        return np.array((0, 0))
    if np.arccos(np.dot((P - A) / np.linalg.norm(P - A), (B - A) / np.linalg.norm(B - A))) > np.pi / 2:
        # beyond A
        return A - P
    if np.arccos(np.dot((P - B) / np.linalg.norm(P - B), (A - B) / np.linalg.norm(A - B))) > np.pi / 2:
        # beyond B
        return B - P

    # in between
    perp = np.array((B[1] - A[1], A[0] - B[0]))
    return perp / np.linalg.norm(perp) * np.cross(A-B, A-P) / np.linalg.norm(B-A)

def segment_normal(l1, l2, ray=False):
    (a, b) = l1
    (c, d) = l2
    normal = np.array((b[1] - a[1], a[0] - b[0]))
    t = np.dot(normal, a-c) / np.dot(normal, d-c)
    return 0 <= t and (ray or t <= 1)

# ''' segments_intersect: np.array[2] * np.array[2] -> boolean '''
# def segments_intersect(ray, line):
#     ''' Finds if the ray ray intersects the line segment line'''
#     return segment_normal(ray, line) and segment_normal(line, ray, ray=True)

''' segments_intersect: np.array[2] * np.array[2] -> boolean '''
def segments_intersect(l1, l2):
    ''' Finds if the line segments intersects'''
    return segment_normal(l1, l2) and segment_normal(l2, l1)

''' segments_for_polygon: [(float, float)] -> [np.array[2]] '''
def segments_for_polygon(polygon):
    ''' Returns a list of line segments making up the polygon '''
    return [(np.array(polygon[i]), np.array(polygon[i+1])) for i in range(0, len(polygon) - 1)] + [(np.array(polygon[-1]), np.array(polygon[0]))]

def segments_for_outline(outline):
    return [(np.array(outline[i]), np.array(outline[i+1])) for i in range(0, len(outline) - 1)] 

''' bounding_box_check: [(float, float)] * np.array[2] -> boolean '''
def bounding_box_check(polygon, point):
    ''' Returns true iff point is in the bounding box of polygon '''
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]

    return min(xs) <= point[0] <= max(xs) and min(ys) <= point[1] <= max(ys)

''' polygon_intersect: [(float, float)] * np.array[2] -> boolean '''
def polygon_intersect(polygon, point):
    ''' Returns true iff point is in polygon '''
    if not bounding_box_check(polygon, point): return False
    ray = (point, point + np.array((1, 0)))
    segments = segments_for_polygon(polygon)
    segment_intersections = [segments_intersect(ray, s) for s in segments]
    count = 0
    for b in segment_intersections: 
        if b:
            count += 1
    return count % 2 != 0

''' find_nearest: [np.array[2]] * np.array[2] -> np.array[3] '''
def find_nearest(segments, pos2d):
    ''' Returns a 3D vector from point to the nearest line segment in segments '''
    vectors = [vector_to_line(s[0], s[1], pos2d) for s in segments]
    if len(vectors) == 0: return None
    m = vectors[0]
    s = segments[0]
    for i in range(1, len(vectors)):
        if np.linalg.norm(m) > np.linalg.norm(vectors[i]):
            m = vectors[i]
            s = segments[i]
    return (s, np.array((m[0], 0, m[1])))


SERIAL_PER_TORQUE = 1000
# COUNTS_PER_INCH = 372.14285714285717 * 0.0393701 * 58/6000
COUNTS_PER_MM = 6700 / 6000 

class Motor():
    ''' Represents a single motor. '''
    def __init__(self, position, torque = 0):
        self.line = 0
        self.torque = torque
        self.pos = position

    def drawing_coords(self):
        return convert3d(self.pos)

    def serial_output(self):
        if math.isnan(self.torque): return b'00'
        val = min(255, max(0, int(self.torque * SERIAL_PER_TORQUE)))
        digit0 = val % 16
        digit1 = (val // 16) % 16
        # digit2 = (val // 16**2) % 16
        # digit3 = (val // 16**3) % 16
        x = ''.join(["0123456789abcdef"[d] for d in (digit1, digit0)])
        return x.encode('utf-8')

    def serial_input(self, data):
        self.line = int(data, 16) / COUNTS_PER_MM

class Controller():
    ''' Represents the entire kinematics controller. '''
    def __init__(self, motors, ser):
        self.motors = motors
        self.polygons = []
        self.outline = []
        self.valid_kinematics = False
        self._position = []
        self.position_set = False
        self.ser = ser

    def update_serial(self):
        motor_index = {b'A': 0, b'B': 1, b'C': 2}

        if not self.ser: return

        d = {}
        while self.ser.in_waiting > 0:
            c = self.ser.read()
            if c in motor_index:
                data = self.ser.read(4)
                motor_to_update = self.motors[motor_index[c]]
                motor_to_update.serial_input(data)
                d[c] = data
        print(d)
        
        msg = self.motors[0].serial_output() + self.motors[1].serial_output() + self.motors[2].serial_output() + b't'
        self.ser.write(msg)
        self.ser.flush()

        self.position_set = False

    ''' set_polygons: [(float, float)] -> void '''
    def set_polygons(self, polygons):
        ''' Sets the current polygons, invalidating the current kinematics. '''
        self.polygons = polygons
        self.valid_kinematics = False

    def set_outline(self, outline):
        self.outline = outline
        self.valid_kinematics = False

    ''' set_position: np.array[3] -> void '''
    def set_position(self, position):
        ''' Forces the controller to a specific position, invalidating the current kinematics. '''
        for m in self.motors:
            m.line = np.linalg.norm(position - m.pos)
        self.valid_kinematics = False
        self._position = []
        self.position_set = False
        
    ''' position: () -> np.array[3] '''
    def position(self):
        ''' Returns the current position, based on the lengths of the motor lines. '''
        if self.position_set: return self._position

        # print("position: ", self.motors[0].line, self.motors[1].line, self.motors[2].line)

        # assumes 3 motors
        # https://en.wikipedia.org/wiki/Trilateration

        (p1, p2, p3) = [m.pos for m in self.motors]
        # print(p1, p2, p3)
        e_x = (p2 - p1) / (np.linalg.norm(p2 - p1))
        i = np.dot(e_x, p3 - p1)
        e_y = (p3 - p1 - i*e_x) / (np.linalg.norm(p3 - p1 - i*e_x))
        e_z = np.cross(e_x, e_y)
        d = np.linalg.norm(p2 - p1)
        j = np.dot(e_y, p3 - p1)

        (r1, r2, r3) = (self.motors[i].line for i in (0, 1, 2))
        # print(r1, r2, r3)

        x = (r1*r1 - r2*r2 + d*d)/(2*d)
        # print(r1, r2, d)
        y = (r1*r1 - r3*r3 + i*i + j*j)/(2*j) - i*x/j
        # print(x, r1, r3, i, j)
        z = -(r1*r1 - x*x - y*y)**0.5
        almost_position = p1 + x*e_x + y*e_y
        # print(r1, almost_position[0], almost_position[1])


        self.position_set = True
        self._position = p1 + x*e_x + y*e_y + z*e_z
        return self._position

    ''' force: () -> np.array[3] '''
    def force(self):
        ''' Returns the current force, based on the motor torques and lines. '''
        dv = np.array((0.0, 0.0, 0.0))
        pos = self.position()
        for m in self.motors:
            v = m.pos - pos
            vec = v / np.linalg.norm(v)
            dv += vec * m.torque
        return dv

    ''' set_force: np.array[3] -> () '''
    def set_force(self, f):
        ''' Forces the controller to a particular force, validating the kinematics. '''
        pos = self.position()
        M = np.column_stack((normalize(m.pos - pos) for m in self.motors))
        try:
            torques = np.linalg.solve(M, f)
            for i in range(len(self.motors)):
                # self.motors[i].torque = max(0, torques[i])
                self.motors[i].torque = torques[i]
        except np.linalg.linalg.LinAlgError as e:
            print("singular matrix")
            for m in self.motors: m.torque = 0
        self.valid_kinematics = True

    ''' set_correct_force: () -> void '''
    def set_correct_force(self):
        ''' Finds the correct direction for the controller to force to, validating the kinematics. '''
        pos = self.position()
        pos2d = np.array((pos[0], pos[2]))
        polys = [p for p in self.polygons if polygon_intersect(p, pos2d)]
        if len(polys) == 0:
            displacement = np.array((0, 0, 0))
        else:
            displacement = find_nearest(segments_for_polygon(polys[0]), pos2d)

        if displacement[2] > 0: 
            # If we're on the backside of a wall, we can't force out of it
            force = np.array((0, 0, 0))
        elif np.linalg.norm(displacement) < 0.001:
            # Too little force
            force = np.array((0, 0, 0))
        else: 
            # Convert the displacement into a force
            # We take the square root of the magnitude of the displacement to make things smoother
            force = displacement / np.sqrt(np.linalg.norm(displacement))

        self.set_force(force * 0.5)

    def move_from_outline(self):
        pos = self.position()
        accum = np.array((0, 0, 0)) if pos[1] > 0 else np.array((0, 1, 0))
        pos2d = np.array((pos[0], pos[2]))
        if len(self.outline) == 0:
            self.set_force(accum + np.array((0, 0, 0)))
            return

        segs = segments_for_outline(self.outline)

        tpl = find_nearest(segs, pos2d)
        if not tpl: 
            self.set_force(accum + np.array((0, 0, 0)))
            return

        (s, displacement) = tpl

        if not any(segments_intersect(s0, (np.array((0, 0)), pos2d)) for s0 in segs): 
            # If we're on the backside of a wall, we can't force out of it
            # print("not intersecting")
            force = np.array((0, 0, 0))
        elif np.linalg.norm(displacement) < 0.001:
            # Too little force
            force = np.array((0, 0, 0))
        else: 
            # Convert the displacement into a force
            # We take the square root of the magnitude of the displacement to make things smoother
            force = displacement / np.sqrt(np.linalg.norm(displacement))

        self.set_force(accum + force * 0.5)


    ''' draw: pygame.screen -> () '''
    def draw(self, screen):
        ''' Draws a picture of the controller, in x-z projection, on to the screen. '''
        (x, z) = convert3d(self.position())
        x = max(0, x)
        z = max(0, z)
        pygame.draw.rect(screen, (0, 0, 0), (max(0, x-10), max(0, z-10), 20, 20), 0)

        pygame.draw.aaline(screen, (0, 0, 0), (x, z), (self.motors[0].drawing_coords()))
        pygame.draw.aaline(screen, (0, 0, 0), (x, z), (self.motors[1].drawing_coords()))
        pygame.draw.aaline(screen, (0, 0, 0), (x, z), (self.motors[2].drawing_coords()))

        (dx, dy, dz) = self.force()
        pygame.draw.line(screen, (255, 0, 0), (x, z), (max(0, x + dx * 10), max(0, z - dz*10)), 5)







