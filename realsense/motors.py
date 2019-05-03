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

FORCE_TO_AMPS = 0.1
ENC_TO_CM = [
    # 0.003, 0.003, 0.003
    50.8 / 22123.9755859375,
    50.8 / 22682.947509765625,
    50.8 / 23022.3447265625
]


def unit(vec):
    return vec / np.linalg.norm(vec)


def trilaterate(encs):
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
        MIDDLE_LOC,
        args=([M1_LOC, M2_LOC, M3_LOC], encs),
        method='Nelder-Mead',
        options={
            'fatol': 1e-5,
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


class Motors():
    def __init__(self):
        self.drive0 = odrive.find_any(serial_number="20763389304B")
        print("found drive0")
        self.drive1 = odrive.find_any(serial_number="208737923548")
        print("found drive1")

        self.motors = [self.drive0.axis1, self.drive1.axis0, self.drive1.axis1]
        self.checkers = [
            EncoderChecker(motor, constant)
            for (motor, constant) in zip(self.motors, ENC_TO_CM)
        ]
        self.encoder_checking = EncoderCheckingThread(self.checkers)

    def start(self):
        set_up_motors(self.motors)
        self.encoder_checking.start()
        # atexit.register(self.on_exit)

    def position(self):
        return trilaterate([checker.get_value() for checker in self.checkers])

    def create_force(self, force):
        pos = self.position()
        forces = []
        for (motor, motor_pos) in zip(self.motors, MOTOR_LOCS):
            # print(self.motors)
            tangent = unit(pos - motor_pos)
            component = -np.dot(tangent, force)
            forces += [component]
            if component > 0:
                motor.controller.current_setpoint = min(
                    10 + component * FORCE_TO_AMPS, 35)
            else:
                motor.controller.current_setpoint = 10
        print(forces)

    def on_exit(self):
        for motor in self.motors:
            motor.controller.current_setpoint = 0