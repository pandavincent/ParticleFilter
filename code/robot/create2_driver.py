"""
Module to control the Create2 using the serial interface
"""

import serial
import struct
import time
from create2 import *
from .gpio import Gpio


class Create2Driver:
    """Class to control the iRobot Create2 robot.

    This class supports only a subset of the functionality described in the documentation:
    http://www.irobotweb.com/~/media/MainSite/PDFs/About/STEM/Create/iRobot_Roomba_600_Open_Interface_Spec.pdf?la=en
    It supports driving as well as getting sensors using the streaming mode.
    """

    def __init__(self, serial_port, brc_pin=None):
        """Constructor.

        Args:
            serial_port (string): device file for the serial port.
            brc_pin (integer): GPIO pin number for the BRC pin used to wake the Create2 up. Use None if not
                               connected.
        """
        self._connection = serial.Serial(serial_port, baudrate=115200, timeout=1)
        self._buffer = bytes()
        if brc_pin is not None:
            # Pulse the BRC pin to wake up Create
            self._brc = Gpio(brc_pin)
            self._brc.set_direction("high")
            self._brc.set_value(1)
            time.sleep(0.02)
            self._brc.set_value(0)
            time.sleep(0.02)
            self._brc.set_value(1)
            # Wait until ready
            time.sleep(0.4)

    def start(self):
        """
        This command starts the OI. You must always send the Start command before sending any other commands to the OI.

        Available in modes: Passive, Safe, or Full; Changes mode to: Passive.

        Roomba beeps once to acknowledge it is starting from "off" mode.
        """
        self._send(">B", (Op.Start,))
        time.sleep(0.02)  # wait ~20ms for mode changes

    def reset(self):
        """
        This command resets the robot, as if you had removed and reinserted the battery.

        Available in modes: Always available; Changes mode to: Off.

        You will have to call start() again to re-enter Open Interface mode.
        """
        self._send(">B", (Op.Reset,))

    def stop(self):
        """
        This command stops the OI. All streams will stop and the robot will no longer respond to commands.
        Use this command when you are finished working with the robot.

        Available in modes: Passive, Safe, or Full; Changes mode to: Off. Roomba plays a song to acknowledge it is exiting the OI.
        """
        self._send(">B", (Op.Stop,))
        time.sleep(0.02)  # wait ~20ms for mode changes

    def safe(self):
        """
        This command puts the OI into Safe mode, enabling user control of Roomba. It turns off all LEDs. The OI
        can be in Passive, Safe, or Full mode to accept this command. If a safety condition occurs Roomba reverts automatically to Passive mode.

        Available in modes: Passive, Safe, or Full; Changes mode to: Safe.
        """
        self._send(">B", (Op.Safe,))
        time.sleep(0.02)  # wait ~20ms for mode changes

    def full(self):
        """
        This command gives you complete control over Roomba by putting the OI into Full mode, and turning off
        the cliff, wheel-drop and internal charger safety features. That is, in Full mode, Roomba executes any
        command that you send it, even if the internal charger is plugged in, or command triggers a cliff or wheel
        drop condition.

        Available in modes: Passive, Safe, or Full; Changes mode to: Full.

        Note: Use start() to change the mode to Passive.
        """
        self._send(">B", (Op.Full,))
        time.sleep(0.02)  # wait ~20ms for mode changes

    def power(self):
        """
        This command powers down Roomba. The OI can be in Passive, Safe, or Full mode to accept this
        command.

        Available in modes: Passive, Safe, or Full; Changes mode to: Passive.
        """
        self._send(">B", (Op.Power,))
        time.sleep(0.02)  # wait ~20ms for mode changes

    def clean(self):
        """
        This command starts the default cleaning mode. This is the same as pressing Roomba's Clean button,
        and will pause a cleaning cycle if one is already in progress.

        Available in modes: Passive, Safe, or Full; Changes mode to: Passive.
        """
        self._send(">B", (Op.Clean,))

    def max(self):
        """
        This command starts the Max cleaning mode, which will clean until the battery is dead. This command
        will pause a cleaning cycle if one is already in progress.

        Available in modes: Passive, Safe, or Full; Changes mode to: Passive.
        """
        self._send(">B", (Op.Max,))

    def spot(self):
        """
        This command starts the Spot cleaning mode. This is the same as pressing Roomba's Spot button, and
        will pause a cleaning cycle if one is already in progress.

        Available in modes: Passive, Safe, or Full; Changes mode to: Passive.
        """
        self._send(">B", (Op.Spot,))

    def seek_dock(self):
        """
        This command directs Roomba to drive onto the dock the next time it encounters the docking beams.
        This is the same as pressing Roomba's Dock button, and will pause a cleaning cycle if one is already in
        progress.

        Available in modes: Passive, Safe, or Full; Changes mode to: Passive.
        """
        self._send(">B", (Op.SeekDock,))

    def drive(self, velocity_in_mm_per_sec, radius_in_mm):
        """
        This command controls Roomba's drive wheels.

        Note: Internal and environmental restrictions may prevent Roomba from accurately carrying out some drive
        commands. For example, it may not be possible for Roomba to drive at full speed in an arc with a large
        radius of curvature.

        Available in modes: Safe or Full; Changes mode to: No Change.

        Args:
            velocity_in_mm_per_sec (integer): -500 - 500 mm/sec. Positive value mean forward and negative backwards.
            radius_in_mm (integer): -2000 - 2000 mm.
                The radius is measured from the center of the turning circle to the center of Roomba.
                Special cases:
                    32768 or 32767: Go straight.
                    -1: Turn in place clockwise.
                    1: Turn in place counterclockwise.
        """
        self._send(">Bhh", (Op.Drive, velocity_in_mm_per_sec, radius_in_mm))

    def drive_direct(self, right_wheel_velocity_in_mm_per_sec, left_wheel_velocity_in_mm_per_sec):
        """
        This command lets you control the forward and backward motion of Roomba's drive wheels
        independently.

        Available in modes: Safe or Full; Changes mode to: No Change.

        Args:
            right_wheel_velocity_in_mm_per_sec (integer): -500 - 500 mm.s. Positive means forward.
            left_wheel_velocity_in_mm_per_sec (integer): -500 - 500 mm/s. Positive means forward.
        """
        self._send(">Bhh", (Op.DriveDirect, right_wheel_velocity_in_mm_per_sec, left_wheel_velocity_in_mm_per_sec))

    def drive_pwm(self, right_pwm, left_pwm):
        """
        This command lets you control the raw forward and backward motion of Roomba's drive wheels
        independently.

        Available in modes: Safe or Full; Changes mode to: No Change

        Args:
            right_pwm (integer): -500 - 500. Positive means forward.
            left_pwm (integer): -500 - 500. Positive means forward.
        """
        self._send(">Bhh", (Op.DrivePwm, right_pwm, left_pwm))

    def motors(self, enable_side_brush, enable_vacuum, enable_main_brush,
               side_brush_clockwise=False, main_brush_outward=False):
        data = (main_brush_outward << 4) \
               | (side_brush_clockwise << 3) \
               | (enable_main_brush << 2) \
               | (enable_vacuum << 1) \
               | (enable_side_brush << 0)
        self._send(">BB", (Op.Motors, data))

    def pwm_motors(self, main_brush_pwm, side_brush_pwm, vacuum_pwm):
        self._send(">Bbbb", (Op.PwmMotors, main_brush_pwm, side_brush_pwm, vacuum_pwm))

    def leds(self, debris, spot, dock, check_robot, power_led_color, power_led_intensity):
        data = (check_robot << 3) \
               | (dock << 2) \
               | (spot << 1) \
               | (debris << 0)
        self._send(">BBBB", (Op.Leds, data, power_led_color, power_led_intensity))

    def digits_leds_ascii(self, data):
        data_all = data + b'    '
        self._send(">Bbbbb", (Op.DigitsLedsAscii, data_all[0], data_all[1], data_all[2], data_all[3]))

    def start_stream(self, sensor_ids):
        self._connection.write(struct.pack(">Bb", Op.Stream, len(sensor_ids)))
        for sensor_id in sensor_ids:
            self._connection.write(struct.pack(">b", sensor_id))

    def update(self):
        self._buffer += self._connection.read(self._connection.inWaiting())
        i = 0
        state = None
        while i < len(self._buffer) - 1:
            if self._buffer[i] == 19:
                size = self._buffer[i+1]
                # print("found 19 " + str(size))
                if len(self._buffer) > size + i + 2:
                    # print("long enough")
                    # check checksum
                    checksum = 0
                    for j in range(i, size+i+3):
                        checksum += self._buffer[j]

                    if (checksum & 0xFF) == 0:
                        # parse packet
                        pos = i + 2
                        state = State()
                        while pos < i + size + 2:
                            sensor = self._buffer[pos]
                            pos += 1
                            if sensor == Sensor.OIMode:
                                fmt = ">B"
                                state.mode, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.ChargingState:
                                fmt = ">B"
                                state.chargingState, = struct.unpack_from(fmt, self._buffer, pos)
                                state.chargingState = ChargingState(state.chargingState)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.Voltage:
                                fmt = ">H"
                                state.voltageInMV, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.Current:
                                fmt = ">h"
                                state.currentInMA, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.Temperature:
                                fmt = ">B"
                                state.temperatureInDegCelcius, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.BatteryCharge:
                                fmt = ">H"
                                state.batteryChargeInMAH, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.BatteryCapacity:
                                fmt = ">H"
                                state.batteryCapacityInMAH, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.CliffLeftSignal:
                                fmt = ">H"
                                state.cliffLeftSignalStrength, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.CliffFrontLeftSignal:
                                fmt = ">H"
                                state.cliffFrontLeftSignalStrength, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.CliffFrontRightSignal:
                                fmt = ">H"
                                state.cliffFrontRightSignalStrength, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.CliffRightSignal:
                                fmt = ">H"
                                state.cliffRightSignalStrength, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LeftEncoderCounts:
                                fmt = ">h"
                                state.leftEncoderCounts, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.RightEncoderCounts:
                                fmt = ">h"
                                state.rightEncoderCounts, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.InfraredCharacterOmni:
                                fmt = ">B"
                                state.infraredCharacterOmni, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.InfraredCharacterLeft:
                                fmt = ">B"
                                state.infraredCharacterLeft, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.InfraredCharacterRight:
                                fmt = ">B"
                                state.infraredCharacterRight, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpLeftSignal:
                                fmt = ">H"
                                state.lightBumpLeftSignal, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpFrontLeftSignal:
                                fmt = ">H"
                                state.lightBumpFrontLeftSignal, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpCenterLeftSignal:
                                fmt = ">H"
                                state.lightBumpCenterLeftSignal, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpCenterRightSignal:
                                fmt = ">H"
                                state.lightBumpCenterRightSignal, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpFrontRightSignal:
                                fmt = ">H"
                                state.lightBumpFrontRightSignal, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            elif sensor == Sensor.LightBumpRightSignal:
                                fmt = ">H"
                                state.lightBumpRightSignal, = struct.unpack_from(fmt, self._buffer, pos)
                                pos += struct.calcsize(fmt)
                            else:
                                print(str.format("Don't know {}", id))

                        # return state
                    else:
                        self._buffer = bytes()
                        print("Checksum incorrect!")

                    # delete parsed portion of buffer
                    self._buffer = self._buffer[size+i:]
                    i = -1
            i += 1

        return state

    def _send(self, fmt, data):
        command = struct.pack(fmt, *data)
        self._connection.write(command)
