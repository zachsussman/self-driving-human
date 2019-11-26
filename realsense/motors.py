import odrive
from odrive.enums import *
import numpy as np
import time
from threading import Thread, Lock
import scipy.optimize
import atexit

M1_LOC = np.array([50.8, 50.8, 0])
M2_LOC = np.array([0, 0, 0])
M3_LOC = np.array([0, 50.8, 0])
MOTOR_LOCS = [M1_LOC, M2_LOC, M3_LOC]

MIDDLE_LOC = sum(MOTOR_LOCS) / 3
MIDDLE_LOC[2] = 1

FORCE_TO_AMPS = 0.008
ENC_TO_CM = [
    # 0.003, 0.003, 0.003
    50.8 / 22123.9755859375,
    50.8 / 22682.947509765625,
    50.8 / 23022.3447265625
]


def unit(vec):
    n = np.linalg.norm(vec)
    if n == 0:
        return vec.astype(float)
    return (vec / np.linalg.norm(vec)).astype(float)


def trilaterate_analytically(encs):
    (p1, p2, p3) = MOTOR_LOCS

    e_x = (p2 - p1) / (np.linalg.norm(p2 - p1))
    i = np.dot(e_x, p3 - p1)
    e_y = (p3 - p1 - i * e_x) / (np.linalg.norm(p3 - p1 - i * e_x))
    e_z = np.cross(e_x, e_y)
    d = np.linalg.norm(p2 - p1)
    j = np.dot(e_y, p3 - p1)

    (r1, r2, r3) = encs

    x = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    y = (r1 * r1 - r3 * r3 + i * i + j * j) / (2 * j) - i * x / j
    z = -(r1 * r1 - x * x - y * y)**0.5
    almost_position = p1 + x * e_x + y * e_y
    return p1 + x * e_x + y * e_y + z * e_z


def trilaterate(encs, guess=MIDDLE_LOC):
    p = trilaterate_analytically(encs)
    if not np.isnan(p[0]):
        return p

    print("Position fail")

    def mse(x, locations, distances):
        ret = 0.0
        for l, d in zip(locations, distances):
            calc = np.sqrt((x[0] - l[0])**2 + (x[1] - l[1])**2 +
                           (x[2] - l[2])**2)
            ret += (calc - d)**2
        # print("distance to", x, "is", ret)
        return ret

    return scipy.optimize.minimize(
        mse,
        guess,
        args=([M1_LOC, M2_LOC, M3_LOC], encs),
        options={
            'gtol': 1e-5,
            'maxiter': 1e+7
        }).x


class EncoderChecker():
    def __init__(self, motor, conv_constant):
        self.motor = motor
        self.min = 10000000
        self.current = None
        self.update_lock = Lock()
        self.conv_constant = conv_constant

    def run(self):
        self.update_lock.acquire()
        self.current = -self.motor.encoder.pos_estimate
        if self.min > self.current:
            self.min = self.current
        self.update_lock.release()

    def get_value(self):
        self.update_lock.acquire()
        ret_val = self.current - self.min
        self.update_lock.release()
        return ret_val * self.conv_constant


class EncoderCheckingThread(Thread):
    def __init__(self, checkers):
        super().__init__()
        self.checkers = checkers
        self.daemon = True

    def run(self):
        while True:
            for checker in self.checkers:
                checker.run()
            time.sleep(0.01)


def set_up_motors(motors):
    # for motor in motors:
    # motor.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    # while any([motor.current_state != AXIS_STATE_IDLE for motor in motors]):
    # time.sleep(0.1)
    for motor in motors:
        motor.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        # motor.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        motor.controller.current_setpoint = 10


class Motor():
    def __init__(self, serial_number, axis_number, cm_constant):
        self.drive = odrive.find_any(serial_number=serial_number)
        if axis_number == 0:
            self.motor = self.drive.axis0
        else:
            self.motor = self.drive.axis1

        self.checker = EncoderChecker(self.motor, cm_constant)
        self.encoder_checking = EncoderCheckingThread([self.checker])

    def start(self):
        self.motor.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.motor.controller.current_setpoint = 10

    def get_value(self):
        return self.checker.get_value()

    def set_force(self, force):
        self.motor.controller.current_setpoint = force


class MotorFake():
    def __init__(self, _1, _2, _3):
        self.length = 60

    def start(self):
        pass

    def set_value(self, length):
        self.length = length

    def get_value(self):
        return self.length + np.random.normal(0, 0.1)

    def set_force(self, force):
        pass


class Motors():
    def __init__(self):
        # self.drive0 = odrive.find_any(serial_number="20763389304B")
        # print("found drive0")
        # self.drive1 = odrive.find_any(serial_number="208737923548")
        # print("found drive1")

        self._position = None

    def start(self):
        self.motors = [
            Motor("208839864D4D", 1, ENC_TO_CM[0]),
            Motor("208839864D4D", 0, ENC_TO_CM[1]),
            Motor("20693597524B", 0, ENC_TO_CM[2])
        ]
        for motor in self.motors:
            motor.start()
        # atexit.register(self.on_exit)

    def position(self):
        guess = MIDDLE_LOC
        if self._position is not None:
            guess = self._position
        self._position = trilaterate(
            [motor.get_value() for motor in self.motors], guess)
        return self._position

    def create_force(self, force, pos):
        positions = np.hstack(
            [[unit(pos - motor_pos) for motor_pos in MOTOR_LOCS]])
        components = np.linalg.lstsq(positions, force)[0]
        for motor, component in zip(self.motors, components):
            if component > 0:
                motor.set_force(min(8 + component * FORCE_TO_AMPS, 27))
            else:
                motor.set_force(8)
        # for (motor, motor_pos) in zip(self.motors, MOTOR_LOCS):
        #     tangent = unit(pos - motor_pos)
        #     component = -np.dot(tangent, force)
        #     forces += [component]
        #     if component > 0:
        #         motor.set_force(min(8 + component * FORCE_TO_AMPS, 27))
        #     else:
        #         motor.set_force(8)

    def on_exit(self):
        for motor in self.motors:
            motor.set_force(0)

    def shutdown(self):
        for motor in self.motors:
            motor.set_force(0)


class MotorsFake(Motors):
    def __init__(self):
        super().__init__()

    def start(self):
        self.motors = [
            MotorFake("20763389304B", 1, ENC_TO_CM[0]),
            MotorFake("208737923548", 0, ENC_TO_CM[1]),
            MotorFake("208737923548", 1, ENC_TO_CM[2])
        ]

    def set_position(self, position):
        for (motor, motor_pos) in zip(self.motors, MOTOR_LOCS):
            motor.set_value(np.linalg.norm(position - motor_pos))