"""
Module to control the Create2 in V-REP.
"""

from vrep import vrep as vrep
import math
from create2 import *


class Create2Vrep:
    """Class to control the iRobot Create2 robot in V-REP.

    This implements a subset of the functionality of Create2Driver. Please see the documentation
    there for a description of the various methods.
    """

    def __init__(self, client_id):
        self._clientID = client_id
        # query wheel joints
        rc, self._rightWheelJoint = vrep.simxGetObjectHandle(self._clientID, "right_wheel_joint",
                                                             vrep.simx_opmode_oneshot_wait)
        rc, self._leftWheelJoint = vrep.simxGetObjectHandle(self._clientID, "left_wheel_joint",
                                                            vrep.simx_opmode_oneshot_wait)

        # query create
        rc, self._create = vrep.simxGetObjectHandle(self._clientID, "create",
                                                    vrep.simx_opmode_oneshot_wait)

        # start the simulation:
        vrep.simxStartSimulation(self._clientID, vrep.simx_opmode_oneshot_wait)

        self._mode = Mode.Off
        self._leftEncoderCount = 0
        self._rightEncoderCount = 0

        self._lastPosRight = None
        self._lastPosLeft = None
        self._sensorIDs = None

    def start(self):
        self._mode = Mode.Passive

    def reset(self):
        pass

    def stop(self):
        self._mode = Mode.Off

    def safe(self):
        self._mode = Mode.Safe

    def full(self):
        self._mode = Mode.Full

    def power(self):
        self._mode = Mode.Off

    def drive_direct(self, right_wheel_velocity_in_mm_per_sec, left_wheel_velocity_in_mm_per_sec):
        # vrep: rad/s
        vrep.simxSetJointTargetVelocity(
            self._clientID,
            self._rightWheelJoint,
            right_wheel_velocity_in_mm_per_sec / (Specs.WheelDiameterInMM * math.pi) * 2 * math.pi,
            vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetVelocity(
            self._clientID,
            self._leftWheelJoint,
            left_wheel_velocity_in_mm_per_sec / (Specs.WheelDiameterInMM * math.pi) * 2 * math.pi,
            vrep.simx_opmode_oneshot)

    def digits_leds_ascii(self, data):
        data_all = data + b'    '
        vrep.simxSetStringSignal(self._clientID, "digitsLedsAscii", data_all, vrep.simx_opmode_oneshot)

    def start_stream(self, sensor_ids):
        self._sensorIDs = sensor_ids

    def update(self):
        state = State()
        vrep.simxSynchronousTrigger(self._clientID)

        # Simulate wheel encoders
        rc, pos_right = vrep.simxGetJointPosition(self._clientID, self._rightWheelJoint, vrep.simx_opmode_oneshot_wait)
        rc, pos_left = vrep.simxGetJointPosition(self._clientID, self._leftWheelJoint, vrep.simx_opmode_oneshot_wait)

        if self._lastPosRight is not None:
            dr = pos_right - self._lastPosRight
            dl = pos_left - self._lastPosLeft

            if dr >= 0:
                dr = math.fmod(dr+math.pi, 2*math.pi) - math.pi
            else:
                dr = math.fmod(dr-math.pi, 2*math.pi) + math.pi

            if dl >= 0:
                dl = math.fmod(dl+math.pi, 2*math.pi) - math.pi
            else:
                dl = math.fmod(dl-math.pi, 2*math.pi) + math.pi

            self._rightEncoderCount += dr * Specs.CountsPerRev / 2 / math.pi
            self._leftEncoderCount += dl * Specs.CountsPerRev / 2 / math.pi

            if self._rightEncoderCount > 32768:
                self._rightEncoderCount -= 65547
            if self._rightEncoderCount < -32768:
                self._rightEncoderCount += 65547
            if self._leftEncoderCount > 32768:
                self._leftEncoderCount -= 65547
            if self._leftEncoderCount < -32768:
                self._leftEncoderCount += 65547

        self._lastPosRight = pos_right
        self._lastPosLeft = pos_left

        if Sensor.LeftEncoderCounts in self._sensorIDs:
            state.leftEncoderCounts = int(self._leftEncoderCount)
        if Sensor.RightEncoderCounts in self._sensorIDs:
            state.rightEncoderCounts = int(self._rightEncoderCount)

        # mode
        if Sensor.OIMode in self._sensorIDs:
            state.mode = self._mode

        # battery
        if Sensor.BatteryCharge in self._sensorIDs:
            state.batteryChargeInMAH = 980
        if Sensor.BatteryCapacity in self._sensorIDs:
            state.batteryCapacityInMAH = 1348

        return state

    def sim_get_position(self):
        rc, (x,y,z) = vrep.simxGetObjectPosition(
            self._clientID,
            self._create,
            -1,
            vrep.simx_opmode_oneshot)
        return (x,y)
