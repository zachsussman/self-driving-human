import odrive
from odrive.enums import *
import numpy as np
import time
from threading import Thread, Lock
import scipy.optimize

M1_LOC = np.array([0, 0, 0])
M2_LOC = np.array([1, 0, 0])
M3_LOC = np.array([0, 1, 0])
MOTOR_LOCS = [M1_LOC, M2_LOC, M3_LOC]

MIDDLE_LOC = sum(MOTOR_LOCS) / 3
MIDDLE_LOC[2] = 1

FORCE_TO_AMPS = 1


def unit(vec):
    return vec / np.linalg.norm(vec)


def trilaterate(encs):
    def mse(x, locations, distances):
        ret = 0.0
        for l, d in zip(locations, distances):
            calc = np.sqrt((x[0] - l[0])**2 + (x[1] - l[1])**2)
            ret += (calc - d)**2
        return ret

    return scipy.optimize.minimize(
        mse,
        MIDDLE_LOC,
        args=([M1_LOC, M2_LOC, M3_LOC], encs),
        method='L-BFGS-B',
        options={
            'ftol': 1e-5,
            'maxiter': 1e+7
        }).x


class EncoderChecker():
    def __init__(self, motor):
        self.motor = motor
        self.min = 10000000
        self.current = None
        self.update_lock = Lock()

    def run(self):
        self.update_lock.acquire()
        self.current = self.motor.encoder.config.offset
        if self.min > self.current:
            self.min = self.current
        self.update_lock.release()

    def get_value(self):
        self.update_lock.acquire()
        ret_val = self.current - self.min
        self.update_lock.release()
        return ret_val


class EncoderCheckingThread(Thread):
    def __init__(self, checkers):
        self.checkers = checkers
        self.daemon = True

    def run(self):
        for checker in self.checkers:
            checker.run()


def set_up_motors(motors):
    for motor in motors:
        motor.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    while any([motor.current_state != AXIS_STATE_IDLE for motor in motors]):
        time.sleep(0.1)
    for motor in motors:
        motor.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        motor.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        motor.controller.current_setpoint = 0


class Motors():
    def __init__(self):
        self.drive = odrive.find_any()
        self.motors = [self.drive.axis0, self.drive.axis1, self.drive.axis2]
        self.checkers = [EncoderChecker(motor) for motor in self.motors]
        self.encoder_checking = EncoderCheckingThread(self.checkers)

    def start(self):
        set_up_motors(self.motors)
        self.encoder_checking.start()

    def position(self):
        return trilaterate([checker.get_value() for checker in self.checkers])

    def create_force(self, force):
        pos = self.position()
        for (motor, motor_pos) in zip(self.motors, MOTOR_LOCS):
            tangent = unit(pos - motor_pos)
            component = np.dot(tangent, force)
            if component > 0:
                motor.current_setpoint = component * FORCE_TO_AMPS
            else:
                motor.current_setpoint = 0
