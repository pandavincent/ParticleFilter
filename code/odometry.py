"""
Helper module for Odometry
"""

import create2
import math


class Odometry:
    """This class keeps the current state (x,y,theta) up-to-date based
    on wheel encoder readings.
    Call the update function as frequent as possible with the current
    encoder readings.
    """
    def __init__(self):
        """Constructor.
        """
        self.x = 0
        self.y = 0
        self.theta = 0

        self.last_left_encoder_counts = None
        self.last_right_encoder_counts = None

        self.d_l = create2.Specs.WheelDiameterInMM / 1000
        self.d_r = create2.Specs.WheelDiameterInMM / 1000
        self.n_l = create2.Specs.CountsPerRev
        self.n_r = create2.Specs.CountsPerRev
        self.w = create2.Specs.WheelDistanceInMM / 1000

    def update(self, left_encoder_counts, right_encoder_counts):
        """Update function to keep the state up-to-date

        Args:
            left_encoder_counts (int)
            right_encoder_counts (int)
        """
        if self.last_right_encoder_counts is not None:
            n_l_actual = left_encoder_counts - self.last_left_encoder_counts
            n_r_actual = right_encoder_counts - self.last_right_encoder_counts
            # account for overflow
            if n_l_actual > 32768:
                n_l_actual -= 65535
            if n_r_actual > 32768:
                n_r_actual -= 65535
            if n_l_actual < -32768:
                n_l_actual += 65535
            if n_r_actual < -32768:
                n_r_actual += 65535

            c_l = math.pi * self.d_l / self.n_l
            c_r = math.pi * self.d_r / self.n_r

            delta_l = c_l * n_l_actual
            delta_r = c_r * n_r_actual

            delta_d = (delta_r + delta_l) / 2
            delta_theta = (delta_r - delta_l) / self.w

            self.x += delta_d * math.cos(self.theta)
            self.y += delta_d * math.sin(self.theta)
            self.theta = math.fmod(self.theta + delta_theta, 2 * math.pi)
        self.last_left_encoder_counts = left_encoder_counts
        self.last_right_encoder_counts = right_encoder_counts
