#!/usr/bin/env python
import math


def clamp(value, range):
    return max(min(value, range[1]), range[0])


class PIDController:
    def __init__(self, kp, ki, kd, range_i, range_output, is_angle=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.range_i = range_i
        self.range_output = range_output
        self.previousError = 0.0
        self.integral = 0.0
        self.previousTime = None
        self.isAngle = is_angle

    def update(self, value, target_value, time):
        if self.isAngle:
            error = math.atan2(math.sin(target_value - value), math.cos(target_value - value))
        else:
            error = target_value - value
        p = self.kp * error
        d = 0
        i = 0
        if self.previousTime is not None:
            dt = time - self.previousTime
            if dt > 0:
                d = self.kd * (error - self.previousError) / dt
            self.integral += error * dt
            self.integral = clamp(self.integral, self.range_i)
            i = self.ki * self.integral
        output = p + i + d
        self.previousTime = time
        self.previousError = error
        return clamp(output, self.range_output)
