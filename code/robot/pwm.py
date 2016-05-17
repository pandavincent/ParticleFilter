"""
Module to use ODROID's HW PWM.
"""

import os


class Pwm:
    """Class to use general pulse-width-modulation.

    This class specifically targets the standard linux support for PWM
    as provided by ODROID. More details can be found here:
    http://odroid.com/dokuwiki/doku.php?id=en:c1_hardware_pwm.
    The sysfs interface is described here: https://www.kernel.org/doc/Documentation/pwm.txt.
    It supports setting the value and waiting for a hardware interrupt.
    """

    _folder = '/sys/devices/platform/pwm-ctrl'

    def __init__(self, number=0):
        """Constructor.

        Args:
            number (integer): GPIO pin number.
        """
        self._number = number

    def enable(self):
        """Enables PWM."""
        self._write("enable", "1")

    def disable(self):
        """Disables PWM."""
        self._write("enable", "0")

    def set_frequency(self, frequency_in_hertz):
        """Set the frequency of the pulse width

        Args:
            frequency_in_hertz (integer): frequency in Hertz.
        """
        self._write("freq", str(frequency_in_hertz))

    def set_duty_cycle(self, duty_in_percent):
        """Set the duty cycle of the pulse width

        Args:
            duty_in_percent (float): duty signal in percent (i.e. 0.0 to 100.0)
        """
        self._write("duty", str(int(duty_in_percent / 100 * 1024)))

    def _write(self, name, value):
        with open(os.path.join(self._folder, name + str(self._number)), 'w') as f:
            f.write(value)
