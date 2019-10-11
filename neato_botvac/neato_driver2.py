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
from typing import List, Tuple

import serial
from serial.threaded import LineReader, ReaderThread


def _boolstr(val: bool) -> str:
    return 'on' if val else 'off'


class ResponseReader(LineReader):
    def __init__(self, state_dict, *args, **kwargs):
        super(ResponseReader, self).__init__(*args, **kwargs)
        self.state = state_dict
        self.current = []

    def connection_made(self, transport):
        super(ResponseReader, self).connection_made(transport)
        sys.stdout.write('port opened\n')

    def _command_complete(self):
        print('Received full message')
        print(self.current)
        for line in self.current:
            tokens = line.split(',')
            if len(tokens) > 1:
                self.state[tokens[0]] = tokens[1]
        print(self.state)
        self.current = []

    def handle_line(self, data):
        if data == '\x1a':
            self._command_complete()
        else:
            self.current.append(data)

    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')


class BotvacDriver:
    def __init__(self, port: str = '/dev/ttyACM0', baud: int = 115200):
        self.state = {}
        self._port = serial.Serial(port, baud, timeout=1)
        self._reader = ReaderThread(self._port, self._get_reader)
        self._reader.start()
        self._reader._connection_made.wait()
        if not self._reader.alive:
            raise RuntimeError('connection_lost already called')
        self._protocol = self._reader.protocol
        self._base_width = 240  # mm
        self._max_speed = 300  # mm/s
        self._setTestMode(True)
        time.sleep(0.5)  # TODO should be able to build this into our communications
        self._setLDS(True)

    def __del__(self):
        self._setLDS(False)
        self._setTestMode(False)
        self._protocol.stop()

    def _get_reader(self):
        return ResponseReader(self.state)

    @property
    def baseWidth(self) -> int:
        """Distance between wheel centers, in mm."""
        return self._base_width

    @property
    def maxSpeed(self) -> int:
        """Max speed of a robot wheel, in mm/s."""
        return self._max_speed

    def _setTestMode(self, on: bool) -> None:
        self._protocol.write_line('testmode {}'.format(_boolstr(on)))

    def _setLDS(self, on: bool) -> None:
        self._protocol.write_line('setldsrotation {}'.format(_boolstr(on)))

    def setMotors(self, left: int, right: int) -> None:
        """Set the speed of the left and right wheel motors, in mm/s."""
        pass

    def getMotors(self) -> Tuple[int, int]:
        """Return current left, right encoder values."""
        pass

    def getCharger(self):
        """Return latest battery status."""
        self._protocol.write_line('getcharger')

    def getScanRanges(self) -> List[float]:
        """
        Return latest LDS info as a list of distances in mm.

        It is expected to be 360 points with 1 degree between each, over 0.2 seconds.
        Therefore time between samples is 0.2 / 360 ~= 0.56ms
        """
        pass


def main():
    driver = BotvacDriver()
    while 1:
        driver.getCharger()
        time.sleep(0.2)
        print(driver.state)
    # ser = serial.serial_for_url('/dev/ttyACM0', baudrate=115200, timeout=1)
    # state = {}
    #
    # def getreader():
    #     return ResponseReader(state)
    #
    # with ReaderThread(ser, getreader) as protocol:
    #     while 1:
    #         protocol.write_line('getcharger')
    #         time.sleep(0.2)


if __name__ == '__main__':
    main()
