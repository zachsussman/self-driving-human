import pygame
import numpy as np
import serial

ser = None
try:
    ser = serial.Serial('/dev/ttyUSB0')
except:
    pass

def convert_polys(polys):
    return [[(point['x'], point['y']) for point in poly] for poly in polygons]

polygons = [[(0, 0), (0, 100), (100, 100), (100, 0)], [(200, 200), (200, 300), (300, 300), (300, 200)]]

SCREENSIZE = 600

COUNTS_PER_INCH = 372.14285714285717

def normalize(v):
    return v / np.linalg.norm(v)

def force_normalize(v): 
    n = np.linalg.norm(v)
    if n < 0.00000001: return v
    return v / np.sqrt(n)

class Motor():
    def __init__(self, position, torque = 0):
        self.line = 0
        self.torque = torque
        self.pos = position

    def drawing_coords(self):
        return (self.pos[0], self.pos[2])

class Controller():
    def __init__(self, motors):
        self.motors = motors

    def update_motors(self):
        for m in self.motors:
            m.line = np.linalg.norm(self.pos - m.pos)

    def set_position(self, position):
        for m in self.motors:
            m.line = np.linalg.norm(position - m.pos)
        
    def position(self):
        ''' Assumes:
                3 motors'''
        #https://en.wikipedia.org/wiki/Trilateration

        (p1, p2, p3) = [m.pos for m in self.motors]
        e_x = (p2 - p1) / (np.linalg.norm(p2 - p1))
        i = np.dot(e_x, p3 - p1)
        e_y = (p3 - p1 - i*e_x) / (np.linalg.norm(p3 - p1 - i*e_x))
        e_z = np.cross(e_x, e_y)
        d = np.linalg.norm(p2 - p1)
        j = np.dot(e_y, p3 - p1)

        (r1, r2, r3) = (self.motors[i].line for i in (0, 1, 2))

        x = (r1*r1 - r2*r2 + d*d)/(2*d)
        y = (r1*r1 - r3*r3 + i*i + j*j)/(2*j) - i*x/j
        z = (r1*r1 - x*x - y*y)**0.5

        correct_pos = p1 + x*e_x + y*e_y + z*e_z
        return correct_pos

    def velocity(self):
        dv = np.array((0.0, 0.0, 0.0))
        pos = self.position()
        for m in self.motors:
            v = m.pos - pos
            vec = v / np.linalg.norm(v)
            dv += vec * m.torque
        return dv

    def set_velocity(self, v):
        pos = self.position()
        M = np.column_stack((normalize(m.pos - pos) for m in self.motors))
        torques = np.linalg.solve(M, v)
        for i in range(len(self.motors)):
            self.motors[i].torque = max(0, torques[i])

    def update_kinematics(self):
        self.set_position(self.position() + self.velocity() / 30)


    def draw(self, screen):
        pos = self.position()
        x = pos[0]
        z = pos[2]
        pygame.draw.rect(screen, (0, 0, 0), (x-10, z-10, 20, 20), 0)

        pygame.draw.aaline(screen, (0, 0, 0), (x, z), (self.motors[0].drawing_coords()))
        pygame.draw.aaline(screen, (0, 0, 0), (x, z), (self.motors[1].drawing_coords()))
        pygame.draw.aaline(screen, (0, 0, 0), (x, z), (self.motors[2].drawing_coords()))

        (dx, dy, dz) = self.velocity()
        pygame.draw.line(screen, (255, 0, 0), (x, z), (x + dx * SCREENSIZE/30, z + dz*SCREENSIZE/30), 5)



def vector_to_line(A, B, P):
    """ segment line AB, point P, where each one is an array([x, y]) """
    if all(A == P) or all(B == P):
        return np.array((0, 0))
    if np.arccos(np.dot((P - A) / np.linalg.norm(P - A), (B - A) / np.linalg.norm(B - A))) > np.pi / 2:
        return A - P
    if np.arccos(np.dot((P - B) / np.linalg.norm(P - B), (A - B) / np.linalg.norm(A - B))) > np.pi / 2:
        return B - P

    perp = np.array(((B - A)[1], (A - B)[0]))
    return perp / np.linalg.norm(perp) * np.cross(A-B, A-P)/np.linalg.norm(B-A)


def segment_normal(l1, l2, ray=False):
    (a, b) = l1
    (c, d) = l2
    normal = np.array(((b-a)[1], (a-b)[0]))
    t = np.dot(normal, a-c) / np.dot(normal, d-c)
    # print(l1, l2, t)
    return 0 <= t and (ray or t <= 1)


def sn(a, b, c, d, r=False):
    return segment_normal((np.array(a), np.array(b)), (np.array(c), np.array(d)), ray=r)

def segments_intersect(ray, line):
    return segment_normal(ray, line) and segment_normal(line, ray, ray=True)

def segments_for_polygon(polygon):
    return [(np.array(polygon[i]), np.array(polygon[i+1])) for i in range(0, len(polygon) - 1)] + [(np.array(polygon[-1]), np.array(polygon[0]))]


def polygon_intersect(polygon, point):
    ray = (point, point + np.array((1, 0)))
    segments = segments_for_polygon(polygon)
    segment_intersections = [segments_intersect(ray, s) for s in segments]
    count = 0
    for b in segment_intersections: 
        if b:
            count += 1
    return count % 2 != 0

def find_nearest(segments, con):
    pos = con.position()
    pos2d = np.array((pos[0], pos[2]))
    vectors = [vector_to_line(s[0], s[1], pos2d) for s in segments]
    m = min(vectors, key=lambda v: np.linalg.norm(v))
    return np.array((m[0], 0, m[1]))

def find_correct_direction(polygons, con):
    pos = con.position()
    pos2d = np.array((pos[0], pos[2]))
    polys = [p for p in polygons if polygon_intersect(p, pos2d)]
    if len(polys) == 0:
        return np.array((0, 0, 0))
    else:
        return find_nearest(segments_for_polygon(polys[0]), con)

motor1 = Motor(np.array((0, 0, 0)), 1)
motor2 = Motor(np.array((500, 20, 0)), 1)
motor3 = Motor(np.array((450, 500, 0)), 2)
con = Controller([motor1, motor2, motor3])
con.set_position(np.array((230, 300, 270)))

print(find_correct_direction(polygons, con))
# raise Exception()

def test(position, velocity):
    con.set_position(position)
    print (position, con.position(), np.linalg.norm(position - con.position()))
    if np.linalg.norm(position - con.position()) > 0.001:
        print("Failed", position, con.position())

    con.set_velocity(velocity)
    v = con.velocity()
    if np.linalg.norm(v - velocity) > 0.001:
        print("Failed vel", velocity, v)


test(np.array((300, 300, 100)), np.array((0,0, -1)))



def main():
    pygame.init()
    display = (600, 600)
    screen = pygame.display.set_mode(display, 0)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    quit()

        while ser and ser.in_waiting > 0:
            data = ser.read(5)
            motor_index = {'A': 0, 'B': 1, 'C': 2}
            motor_to_update = con.motors[motor_index[data[0]]]
            new_encoder = int(data[1:], 16)
            motor_to_update.line = new_encoder / COUNTS_PER_INCH


        mouse_pos = pygame.mouse.get_pos()
        con.set_position(np.array((mouse_pos[0], 200, mouse_pos[1])))
        con.set_velocity(np.array((0, 0, -1)))
        screen.fill((255, 255, 255))

        vel = find_correct_direction(polygons, con)
        if vel[2] > 0: vel = np.array((0, 0, 0))
        vel = force_normalize(vel)

        con.set_velocity(vel * 0.5)
        pos2d = np.array((con.position()[0], con.position()[2]))
        for poly in polygons:
            color = (0, 128, 0) if polygon_intersect(poly, pos2d) else (0, 0, 128)
            pygame.draw.polygon(screen, color, poly)
        con.draw(screen)

        print(pos2d)


        pygame.display.flip()
        pygame.time.wait(30)

main()






