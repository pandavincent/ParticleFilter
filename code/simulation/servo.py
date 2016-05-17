"""
Module to control a virtual Servo.
"""

from vrep import vrep as vrep
import math


class Servo:
    """
    Class to control a virtual servo in V-REP.
    The servo is modeled as joint, using an integrated position controller in V-REP.
    """
    def __init__(self, client_id):
        """Constructor.

        Args:
            client_id (integer): V-REP client id.
        """
        self._clientID = client_id
        # query objects
        rc, self._joint = vrep.simxGetObjectHandle(self._clientID, "servo_joint", vrep.simx_opmode_oneshot_wait)

    def go_to(self, angle):
        """Go to specified target angle.

        Args:
            angle (float): -90 - 90 degrees. 0 means facing forward. Negative numbers turn to the left.
        """
        vrep.simxSetJointTargetPosition(self._clientID, self._joint, math.radians(angle),
                                        vrep.simx_opmode_oneshot_wait)
