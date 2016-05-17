"""
Module with factory methods for different objects (either real or simulation)
"""


class FactoryCreate:
    """Class to create objects which are related to the physical iRobot Create2 robot.
    """

    def __init__(self):
        """Constructor.
        """
        from robot import Create2Driver
        self._create = Create2Driver("/dev/ttyS2", 87)
        self._clientID = None

    def close(self):
        """Clean-up
        """
        self._create.drive_direct(0, 0)
        self._create.digits_leds_ascii(bytes("    ", encoding='ascii'))
        self._create.stop()

    def create_create(self):
        """Instantiates a new create robot (only a single one is supported!)

        Returns:
            (robot.Create2Driver) instance of robot.Create2Driver
        """
        return self._create

    def create_time_helper(self):
        """Instantiates a new time object.

        Returns:
            (time) instance of time
        """
        import time
        return time

    def create_sonar(self):
        """Instantiates a new sonar (only a single one is supported!)

        Returns:
            (robot.Sonar) instance of robot.Sonar
        """
        from robot import Sonar
        return Sonar(104)

    def create_servo(self):
        """Instantiates a new servo (only a single one is supported!)

        Returns:
            (robot.Servo) instance of robot.Servo
        """
        from robot import Servo
        return Servo(0)

    def create_virtual_create(self, hostname):
        """Instantiates a new virtual create for visualization (only a single one is supported!)

        Returns:
            (visualization.VirtualCreate) instance of visualization.VirtualCreate
        """
        from vrep import vrep as vrep
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self._clientID = vrep.simxStart(hostname, 19997, True, True, 5000, 5)  # Connect to V-REP

        from visualization import VirtualCreate
        return VirtualCreate(self._clientID)


class FactorySimulation:
    """Class to create objects which are simulated.
    """

    def __init__(self):
        """Constructor.
        """
        from vrep import vrep as vrep
        vrep.simxFinish(-1)  # just in case, close all opened connections
        self._clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to V-REP

        # enable the synchronous mode on the client:
        vrep.simxSynchronous(self._clientID, True)

        # start the simulation:
        vrep.simxStartSimulation(self._clientID, vrep.simx_opmode_oneshot_wait)

    def close(self):
        """Clean-up
        """
        from vrep import vrep as vrep
        # stop the simulation:
        vrep.simxStopSimulation(self._clientID, vrep.simx_opmode_oneshot_wait)
        # close the connection to V-REP:
        vrep.simxFinish(self._clientID)

    def create_create(self):
        """Instantiates a new create robot (only a single one is supported!)

        Returns:
            (simulation.Create2Vrep) instance of simulation.Create2Vrep
        """
        from simulation import Create2Vrep
        return Create2Vrep(self._clientID)

    def create_time_helper(self):
        """Instantiates a new time object.

        Returns:
            (simulation.TimeHelper) instance of simulation.TimeHelper
        """
        from simulation import TimeHelper
        return TimeHelper(self._clientID)

    def create_sonar(self):
        """Instantiates a new sonar (only a single one is supported!)

        Returns:
            (simulation.Sonar) instance of simulation.Sonar
        """
        from simulation import Sonar
        return Sonar(self._clientID)

    def create_servo(self):
        """Instantiates a new servo (only a single one is supported!)

        Returns:
            (simulation.Servo) instance of simulation.Servo
        """
        from simulation import Servo
        return Servo(self._clientID)

    def create_virtual_create(self):
        """Instantiates a new virtual create for visualization (only a single one is supported!)

        Returns:
            (visualization.VirtualCreate) instance of visualization.VirtualCreate
        """
        from visualization import VirtualCreate
        return VirtualCreate(self._clientID)

    def create_kuka_lbr4p(self):
        """Instantiates a new robotic arm (only a single one is supported!)

        Returns:
            (simulation.KukaLBR4PlusVrep) instance of simulation.KukaLBR4PlusVrep
        """
        from simulation import KukaLBR4PlusVrep
        return KukaLBR4PlusVrep(self._clientID)