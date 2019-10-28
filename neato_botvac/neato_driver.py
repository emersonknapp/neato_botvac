#!/usr/bin/env python3
# Copyright 2019 Emerson Knapp
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys
import time
import traceback
from typing import Callable
from typing import List
from typing import NamedTuple
from typing import Optional

import serial
from serial.threaded import LineReader, ReaderThread


def _boolstr(val: bool) -> str:
    return 'on' if val else 'off'


class Encoders(NamedTuple):
    stamp: float
    left: int
    right: int


class BatteryStatus(NamedTuple):
    stamp: float
    voltage: float
    temperature: float
    current: float
    percentage: int


class Scan(NamedTuple):
    stamp: float
    """
    List of distances in mm.

    It is expected to be 360 points with 1 degree between each, over 0.2 seconds.
    Therefore time between samples is 0.2 / 360 ~= 0.56ms
    """
    ranges: List[int]


class BotvacDriverCallbacks(NamedTuple):
    encoders: Optional[Callable[[Encoders], None]]
    battery: Optional[Callable[[BatteryStatus], None]]
    scan: Optional[Callable[[Scan], None]]


class ResponseReader(LineReader):
    """Receive response lines from the botvac and interpret them as structured data."""

    def __init__(self, callbacks, *args, **kwargs):
        super(ResponseReader, self).__init__(*args, **kwargs)
        self.current_stamp = time.time()
        self.current_lines = []
        self.callbacks = callbacks

    def connection_made(self, transport):
        super(ResponseReader, self).connection_made(transport)
        sys.stdout.write('port opened\n')

    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')

    def interpret(self, lines):
        if len(lines) < 3:
            # This is an echo of a command we sent that has no data
            return

        cmd = lines[0].strip('\x1a').split()[0]
        fields = lines[1].split(',')

        if cmd == 'getldsscan':
            result = Scan(
                stamp=self.current_stamp,
                ranges=[int(line.split(',')[1]) for line in lines[2:-1]]
            )
            if self.callbacks.scan:
                self.callbacks.scan(result)
            return

        if len(fields) > 2:
            print('LOTS OF FIELDS')
            print(fields)

        state = {}
        for line in lines[2:]:
            tokens = line.split(',')
            if len(tokens) != 2:
                print('bad value line!')
            else:
                state[tokens[0]] = tokens[1]

        if cmd == 'getmotors':
            result = Encoders(
                stamp=self.current_stamp,
                left=int(state['LeftWheel_PositionInMM']),
                right=int(state['RightWheel_PositionInMM']))
            if self.callbacks.encoders:
                self.callbacks.encoders(result)
            return
        elif cmd == 'getcharger':
            result = BatteryStatus(
                stamp=self.current_stamp,
                voltage=float(state['VBattV']),
                temperature=float(state['BattTempCAvg']),
                current=float(state['Discharge_mAH']),
                percentage=float(state['FuelPercent']))
            if self.callbacks.battery:
                self.callbacks.battery(result)
            return

    def _command_complete(self):
        try:
            self.interpret(self.current_lines)
        except Exception as e:
            print('Interpreting failed: {}'.format(e))
        # clean up
        self.current_lines = []

    def handle_line(self, data):
        if not len(self.current_lines):
            self.current_stamp = time.time()
        if data == '\x1a':
            self._command_complete()
        else:
            self.current_lines.append(data)


class BotvacDriver:
    """Interface to the mobile base and its sensors."""

    def __init__(
        self,
        port: str = '/dev/ttyACM0',
        baud: int = 115200,
        callbacks: Optional[BotvacDriverCallbacks] = None,
    ):
        self._base_width = 240  # mm
        self._max_speed = 300  # mm/s
        self._callbacks = callbacks
        self._stopped = False  # if the robot should not be moving

        self._port = serial.Serial(port, baud, timeout=1)
        self._reader = ReaderThread(self._port, self._get_reader)
        self._reader.start()
        self._reader._connection_made.wait()

        if not self._reader.alive:
            raise RuntimeError('connection_lost already called')

        self._protocol = self._reader.protocol
        self._setTestMode(True)
        time.sleep(0.5)  # TODO should be able to build this into our communications
        self._setLDS(True)

    def __del__(self):
        self._setLDS(False)
        self._setTestMode(False)

    def _get_reader(self):
        return ResponseReader(self._callbacks)

    @property
    def base_width(self) -> int:
        """Distance between wheel centers, in mm."""
        return self._base_width

    @property
    def max_speed(self) -> int:
        """Max speed of a robot wheel, in mm/s."""
        return self._max_speed

    def _setTestMode(self, on: bool) -> None:
        self._protocol.write_line('testmode {}'.format(_boolstr(on)))

    def _setLDS(self, on: bool) -> None:
        self._protocol.write_line('setldsrotation {}'.format(_boolstr(on)))

    def setMotors(self, left_dist: int, right_dist: int, speed: int) -> None:
        """Set motors, distance left & right + speed."""
        """
        The following is a work-around for a bug in the Neato API. The bug is that the
        robot won't stop instantly if a 0-velocity command is sent - the robot
        could continue moving for up to a second. To work around this bug, the
        first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then,
        the zero is sent. This effectively causes the robot to stop instantly.
        """
        if (int(left_dist) == 0 and int(right_dist) == 0 and int(speed) == 0):
            if (not self._stopped):
                self._stopped = True
                left_dist = 1
                right_dist = 1
                speed = 1
        else:
            self._stopped = False

        self._protocol.write_line(
            'setmotor lwheeldist {} rwheeldist {} speed {}'.format(
                left_dist, right_dist, speed))

    def requestEncoders(self):
        """Send a request for the latest motor encoder status."""
        self._protocol.write_line('getmotors')

    def requestBattery(self) -> None:
        """Send a request for latest battery status."""
        self._protocol.write_line('getcharger')

    def requestScan(self) -> None:
        """Send a request for latest LIDAR scan."""
        self._protocol.write_line('getldsscan')


def main():
    driver = BotvacDriver()
    while 1:
        # driver.requestEncoders()
        # driver.requestBattery()
        driver.requestScan()
        time.sleep(1)


if __name__ == '__main__':
    main()
