"""
Module to deal with simulation time.
"""
from vrep import vrep as vrep


class TimeHelper:
    """This class is similar to the default time module of python, however it uses V-REPs simulation time
    rather than real time.
    """
    def __init__(self, client_id):
        """Constructor.

        Args:
            client_id (integer): V-REP client id.
        """
        self._clientID = client_id

    def sleep(self, wait_in_sec):
        """Wait for the specified number of seconds (simulation time).

        The simulation will continue making progress.

        Args:
            wait_in_sec (float): time (in seconds) to continue simulation.
        """
        # query a fake handle just to get correct time afterwards
        vrep.simxGetObjectHandle(self._clientID, "dummy", vrep.simx_opmode_oneshot_wait)
        start = vrep.simxGetLastCmdTime(self._clientID)
        while True:
            vrep.simxSynchronousTrigger(self._clientID)
            time = vrep.simxGetLastCmdTime(self._clientID)
            if start + wait_in_sec * 1000 <= time:
                break

    def time(self):
        """Query current time. Simulation starts at time 0.

        Returns:
            Elapsed simulated seconds.
        """
        # query a fake handle just to get correct time afterwards
        vrep.simxGetObjectHandle(self._clientID, "dummy", vrep.simx_opmode_oneshot_wait)
        t = vrep.simxGetLastCmdTime(self._clientID)
        return t / 1000.0
