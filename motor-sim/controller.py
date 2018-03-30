import pygame


SCREENSIZE = 600

def distance(a, b):
    return sum([(a[i] - b[i])**2 for i in range(0, len(a))])**0.5

def normal(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    length = distance((dx, dy, dz), (0, 0, 0))
    return (dx/length, dy/length, dz/length)


class Motor():
    def __init__(self, position, torque = 0):
        self.line = 0
        self.torque = torque
        self.pos = position

class Controller():
    def __init__(self, motors):
        self.motors = motors
        self.pos = None

    def move_to(self, position):
        self.pos = position
        for m in self.motors:
            m.line = distance(self.pos, m.pos)


    def determine_position(self):
        ''' Assumes:
                3 motors
                motors[0].position = (0, 0, 0)
                motors[1].position = (1, 0, 0)
                motors[2].position = (1, 1, 0)'''
        #https://en.wikipedia.org/wiki/Trilateration

        (r1, r2, r3) = (self.motors[i].line for i in (0, 1, 2))
        d = 1
        i = 1
        j = 1

        x = (r1*r1 - r2*r2 + d*d)/(2*d)
        y = (r1*r1 - r3*r3 + i*i + j*j)/(2*j) - i*x/j
        z = (r1*r1 - x*x - y*y)**0.5



    def velocity(self):
        dx = 0
        dy = 0
        dz = 0
        for m in self.motors:
            vec = normal(m.pos, self.pos)
            dx += vec[0] * m.torque
            dy += vec[1] * m.torque
            dz += vec[2] * m.torque
        return (dx, dy, dz)


    def draw(self, screen):
        x = self.pos[0] * SCREENSIZE
        y = self.pos[1] * SCREENSIZE
        pygame.draw.rect(screen, (0, 0, 0), (x-10, y-10, 20, 20), 0)

        pygame.draw.aaline(screen, (0, 0, 0), (x, y), (0, 0))
        pygame.draw.aaline(screen, (0, 0, 0), (x, y), (SCREENSIZE, 0))
        pygame.draw.aaline(screen, (0, 0, 0), (x, y), (SCREENSIZE, SCREENSIZE))

        (dx, dy, dz) = self.velocity()
        pygame.draw.line(screen, (255, 0, 0), (x, y), (x + dx * 20, y + dy*20), 5)


motor1 = Motor((0, 0, 0), 1)
motor2 = Motor((1, 0, 0), 1)
motor3 = Motor((1, 1, 0), 2)
con = Controller([motor1, motor2, motor3])
con.move_to((0.5, 0.75, 0.5))

def test(position):
    con.move_to(position)
    con.determine_position()
    if distance(position, con.pos) > 0.001:
        print("Failed", position)



def main():
    pygame.init()
    display = (600, 600)
    screen = pygame.display.set_mode(display, 0)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        screen.fill((255, 255, 255))
        con.draw(screen)

        pygame.display.flip()
        pygame.time.wait(15)

main()






