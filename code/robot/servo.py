"""
Module to control a Parallax Servo.
"""

from .pwm import Pwm


class Servo:
    def __init__(self, number):
        """Constructor.

        Args:
            number (integer): PWM number where the servo is connected to.
        """
        self._pwm = Pwm(number)
        # https://www.parallax.com/sites/default/files/downloads/900-00005-Standard-Servo-Product-Documentation-v2.2.pdf
        #  0.75-2.25  ms high pulse, 20 ms intervals
        # i.e. 50 Hertz, and duty cycle: 3.75 percent to 11.25 percent
        self._pwm.set_frequency(50)
        self._pwm.set_duty_cycle(7.5)
        self._pwm.enable()

    def go_to(self, angle):
        self._pwm.set_duty_cycle(7.5 + angle / 90.0 * 3.75)
