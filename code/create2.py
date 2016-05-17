"""
Helper enumerations which are both used for simulation and driver.
"""
from enum import Enum


# noinspection PyClassHasNoInit
class Sensor:
    BumpsAndWheelDrops = 7
    # Wall                      = 8 # Deprecated
    CliffLeft = 9
    CliffFrontLeft = 10
    CliffFrontRight = 11
    CliffRight = 12
    VirtualWall = 13
    WheelOvercurrents = 14
    DirtDetect = 15
    # UnusedByte                = 16
    InfraredCharacterOmni = 17
    Buttons = 18
    Distance = 19  # broken on firmware < 3.3.0
    Angle = 20  # broken on firmware < 3.4.0
    ChargingState = 21  # one of ChargingState
    Voltage = 22  # mV
    Current = 23  # mA
    Temperature = 24  # degC
    BatteryCharge = 25  # mAh
    BatteryCapacity = 26
    # WallSignal                = 27 # Deprecated
    CliffLeftSignal = 28
    CliffFrontLeftSignal = 29
    CliffFrontRightSignal = 30
    CliffRightSignal = 31
    # Unused                    = 32
    # Unused                    = 33
    ChargingSourcesAvailable = 34
    OIMode = 35  # one of Mode
    SongNumber = 36
    SongPlaying = 37
    NumberOfStreamPackets = 38
    RequestedVelocity = 39
    RequestedRadius = 40
    RequestedRightVelocity = 41
    RequestedLeftVelocity = 42
    LeftEncoderCounts = 43
    RightEncoderCounts = 44
    RightBumper = 45
    LightBumpLeftSignal = 46
    LightBumpFrontLeftSignal = 47
    LightBumpCenterLeftSignal = 48
    LightBumpCenterRightSignal = 49
    LightBumpFrontRightSignal = 50
    LightBumpRightSignal = 51
    InfraredCharacterLeft = 52
    InfraredCharacterRight = 53
    LeftMotorCurrent = 54  # mA
    RightMotorCurrent = 55  # mA
    MainBrushMotorCurrent = 56  # mA
    SideBrushMotorCurrent = 57  # mA
    Stasis = 58


# noinspection PyClassHasNoInit
class Op:
    Reset = 7
    Start = 128
    Baud = 129
    Control = 130
    Safe = 131
    Full = 132
    Power = 133
    Spot = 134
    Clean = 135
    Max = 136
    Drive = 137
    Motors = 138
    Leds = 139
    Song = 140
    Play = 141
    Sensors = 142
    SeekDock = 143
    PwmMotors = 144
    DriveDirect = 145
    DrivePwm = 146
    Stream = 148
    QueryList = 149
    PauseResumeStream = 150
    SchedulingLeds = 162
    DigitsLedsRaw = 163
    DigitsLedsAscii = 164
    Buttons = 165
    Schedule = 167
    SetDayTime = 168
    Stop = 173


class ChargingState(Enum):
    NotCharging = 0
    ReconditioningCharging = 1
    FullCharging = 2
    TrickleCharging = 3
    ChargingStateWaiting = 4
    ChargingFaultCondition = 5


class Mode(Enum):
    Off = 0
    Passive = 1
    Safe = 2
    Full = 3


class State:
    def __init__(self):
        pass
        # self.mode = None
        # self.voltageInMV = None
        # self.currentInMA = None
        # self.temperatureInDegCelcius = None
        # self.batteryChargeInMAH = None
        # self.batteryCapacityInMAH = None
        # self.cliffLeftSignalStrength = None
        # self.cliffFrontLeftSignalStrength = None
        # self.cliffFrontRightSignalStrength = None
        # self.cliffRightSignalStrength = None
        # self.leftEncoderCounts = None
        # self.rightEncoderCounts = None


# noinspection PyClassHasNoInit
class Specs:
    CountsPerRev = 508.8
    WheelDiameterInMM = 72.0
    WheelDistanceInMM = 235.0


class InfraredCharacter(Enum):
    DockRedBuoy = 248
    DockGreenBuoy = 244
    DockForceField = 242
    DockRedBuoyGreenBuoy = 252
    DockRedBuoyAndForceField = 250
    DockGreenBuoyAndForceField = 246
    DockRedBuoyAndGreenBuoyAndForceField = 254
