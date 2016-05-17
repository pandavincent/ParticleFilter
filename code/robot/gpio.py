"""
Module to use ODROID's GPIOs.
"""

import os
import select
import time


class Gpio:
    """Class to use general purpose input/output (GPIO) pins.

    This class specifically targets the standard linux support for GPIO
    as provided by ODROID. More details can be found here:
    http://odroid.com/dokuwiki/doku.php?id=en:c1_hardware_pwm.
    The sysfs interface is described here: https://www.kernel.org/doc/Documentation/gpio/sysfs.txt.
    It supports setting the value and waiting for a hardware interrupt.
    """

    def __init__(self, number):
        """Constructor.

        Args:
            number (integer): GPIO pin number.
        """
        self._folder = "/sys/class/aml_gpio/gpio" + str(number)
        self._f = None
        self._po = None
        # check if GPIO is already exported
        if not os.path.exists(self._folder):
            # export the GPIO to make it usable
            with open("/sys/class/aml_gpio/export", 'w') as f:
                f.write(str(number))
            # wait until udev rule updated the access rights
            while not os.access(os.path.join(self._folder, "direction"), os.W_OK):
                time.sleep(0.1)

    def set_direction(self, direction):
        """Set the direction (input/output) of the pin.

        Args:
            direction (string): One of "in", "out", "low", "high".
        """
        self._write("direction", direction)

    def set_value(self, value):
        """Set the current value of the pin (only valid if configured as output.)

        Args:
            value (integer): 0 or 1.
        """
        self._write("value", str(value))

    def set_edge(self, edge):
        """Set the edge trigger for HW interrupt support. Use `wait_for_interrupt` to wait for an interrupt afterwards.

        Args:
            edge (string): One of "none", "rising", "falling", "both"
        """
        self._write("edge", edge)
        self._f = open(os.path.join(self._folder, 'value'), 'r')
        self._po = select.poll()
        self._po.register(self._f, select.POLLPRI)

    def wait_for_interrupt(self, timeout_in_ms=1000):
        """Waits until timeout or interrupt occurs.

        Args:
            timeout_in_ms (integer): maximum time to wait for an interrupt

        Returns:
            None if timeout occurred or the current value of the pin in case the interrupt was triggered.
        """
        events = self._po.poll(timeout_in_ms)
        if not events:
            return None
        else:
            self._f.seek(0)
            res = self._f.read()
            return res

    def _write(self, name, value):
        with open(os.path.join(self._folder, name), 'w') as f:
            f.write(value)
