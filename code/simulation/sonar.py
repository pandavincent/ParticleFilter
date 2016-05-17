"""
Module to control a virtual Sonar.
"""

from vrep import vrep as vrep
import math


class Sonar:
    """
    Class to control a virtual sonar.
    """
    def __init__(self, client_id):
        """Constructor.

        Args:
            client_id (integer): V-REP client id.
        """
        self._clientID = client_id
        # query objects
        rc, self._sensor = vrep.simxGetObjectHandle(self._clientID, "Proximity_sensor", vrep.simx_opmode_oneshot_wait)

    def get_distance(self):
        """Queries the current distance from the sonar.

        We use a proximity sensor in V-REP to model the sonar.

        Returns:
            Distance in m. If there was an error, it returns 3.3 m.
        """
        return_code, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = \
            vrep.simxReadProximitySensor(self._clientID, self._sensor, vrep.simx_opmode_oneshot_wait)
        if detection_state:
            return math.sqrt(
                    math.pow(detected_point[0], 2) +
                    math.pow(detected_point[1], 2) +
                    math.pow(detected_point[2], 2))
        else:
            return 3.3
