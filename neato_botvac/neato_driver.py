#!/usr/bin/env python3
# Copyright (c) 2010 University at Albany. All right reserved.
# Copyright 2019 Emerson Knapp
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__author__ = 'ferguson@cs.albany.edu (Michael Ferguson)'

import threading
import time
from typing import List, Tuple

import serial

BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second


class Botvac():
    """
    Driver class to communicate easily with a Neato Botvac.

    This has only been tested on a D-series.
    """

    def __init__(self, shutdown_check: threading.Event, port: str = '/dev/ttyACM0'):
        self.port = serial.Serial(port, 115200, timeout=0.1)
        if not self.port.isOpen():
            self.err('Failed To Open Serial Port')
            return
        self.info('Open Serial Port {} ok'.format(port))

        # Storage for motor and sensor information
        self.state = {}

        self.stop_state = True
        self.responseData = []
        self.currentResponse = []
        self.stop_requested = shutdown_check
        self.readLock = threading.RLock()

        self.readThread = threading.Thread(target=self.read, daemon=True)
        self.readThread.start()

        self._setTestMode(True)
        # For some reason testmode takes a sec to activate
        # and the rest of the commands won't take if it hasn't
        time.sleep(0.2)
        self.setLed('ledgreen')
        self.setLed('blinkon')
        self.setLDS(True)

        self.base_width = BASE_WIDTH
        self.max_speed = MAX_SPEED

        self.flush()

        self.info('Init Done')

    def __del__(self):
        self.setLDS(False)
        self.setLed('buttonoff')

        time.sleep(0.5)

        self._setTestMode(False)
        self.port.flush()

        self.readThread.join()

        self.port.close()

    def err(self, msg: str):
        print(msg)

    def info(self, msg: str):
        print(msg)

    def _bool_cmd(self, cmd: str, value: bool) -> None:
        self.sendCmd('{} {}'.format(cmd, 'on' if value else 'off'))

    def _setTestMode(self, value: bool) -> None:
        """Turn test mode on/off."""
        self._bool_cmd('testmode', value)

    def setLDS(self, value: bool) -> None:
        self._bool_cmd('setldsrotation', value)

    def requestScan(self) -> None:
        """Ask neato for an array of scan reads."""
        self.sendCmd('getldsscan')

    def getScanRanges(self) -> List[float]:
        """Read values of a scan. Call requestScan first so that values are ready."""
        ranges = []
        angle = 0

        if not self.readTo('AngleInDegrees'):
            self.flush()
            return []

        last = False
        while not last:  # angle < 360:
            try:
                vals, last = self.getResponse()
            except Exception as ex:
                self.err('Exception Reading Neato lidar: ' + str(ex))
                last = True
                vals = []

            vals = vals.split(',')

            # TODO what are magic vals 48 and 57?
            if ((not last) and ord(vals[0][0]) >= 48 and ord(vals[0][0]) <= 57):
                try:
                    a = int(vals[0])
                    r = int(vals[1])
                    e = int(vals[3])

                    # fill in missing scan values
                    while (angle < a):
                        ranges.append(0.0)
                        angle += 1

                    if e == 0:
                        ranges.append(r/1000.0)
                    else:
                        ranges.append(0.0)
                # TODO what exceptions are expected?
                except Exception as ex:
                    self.err('EXCEPTIONAL {}'.format(ex))
                    ranges.append(0.0)
                angle += 1

        if len(ranges) != 360:
            self.info('Missing laser scans: got {} points'.format(len(ranges)))

        return ranges

    def setMotors(self, left: int, right: int, speed: int) -> None:
        """Set motors, distance left & right + speed."""
        """
        The following is a work-around for a bug in the Neato API. The bug is that the
        robot won't stop instantly if a 0-velocity command is sent - the robot
        could continue moving for up to a second. To work around this bug, the
        first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then,
        the zero is sent. This effectively causes the robot to stop instantly.
        """
        if (int(left) == 0 and int(right) == 0 and int(speed) == 0):
            if (not self.stop_state):
                self.stop_state = True
                left = 1
                right = 1
                speed = 1
        else:
            self.stop_state = False

        self.sendCmd(
            'setmotor' +
            ' lwheeldist ' + str(int(left)) +
            ' rwheeldist ' + str(int(right)) +
            ' speed ' + str(int(speed)))

    def getMotors(self) -> Tuple[int, int]:
        """
        Update values for motors in the self.state dictionary.

        Returns current left, right encoder values.
        """
        self.sendCmd('getmotors')

        if not self.readTo('Parameter'):
            self.flush()
            return [0, 0]

        last = False
        while not last:
            try:
                vals, last = self.getResponse()
                values = vals.split(',')
                self.state[values[0]] = float(values[1])
            except Exception as ex:
                self.err('Exception Reading Neato motors: ' + str(ex))

        return (self.state['LeftWheel_PositionInMM'], self.state['RightWheel_PositionInMM'])

    def getAnalogSensors(self) -> None:
        """Update values for analog sensors in the self.state dictionary."""
        self.sendCmd('getanalogsensors')

        if not self.readTo('SensorName'):
            self.flush()
            return

        last = False
        while not last:  # for i in range(len(xv11_analog_sensors)):
            try:
                vals, last = self.getResponse()
                values = vals.split(',')
                self.state[values[0]] = int(values[1])
            except Exception as ex:
                self.err('Exception Reading Neato Analog sensors: ' + str(ex))

    def getDigitalSensors(self) -> Tuple[int, int, int, int]:
        """Update values for digital sensors in the self.state dictionary."""
        self.sendCmd('getdigitalsensors')

        if not self.readTo('Digital Sensor Name'):
            self.flush()
            return [0, 0, 0, 0]

        last = False
        while not last:
            try:
                vals, last = self.getResponse()
                values = vals.split(',')
                self.state[values[0]] = int(values[1])
            except Exception as ex:
                self.err('Exception Reading Neato Digital sensors: ' + str(ex))
        return [
            self.state['LSIDEBIT'], self.state['RSIDEBIT'],
            self.state['LFRONTBIT'], self.state['RFRONTBIT']]

    def getButtons(self):
        raise NotImplementedError('GetButtons not implemented')
        # return [0, 0, 0, 0, 0]

    def getCharger(self) -> None:
        """Update values for charger/battery related info in self.state dictionary."""
        self.sendCmd('getcharger')

        if not self.readTo('Label'):
            self.flush()
            return

        last = False
        while not last:
            vals, last = self.getResponse()
            values = vals.split(',')
            try:
                self.state[values[0]] = float(values[1])
            except Exception as ex:
                self.err('Exception Reading Neato charger info: ' + str(ex))

    def setBacklight(self, value: bool) -> None:
        if value:
            self.sendCmd('setled backlighton')
        else:
            self.sendCmd('setled backlightoff')

    def setLed(self, cmd: str) -> None:
        self.sendCmd('setled %s' % cmd)

    def setLED(self, cmd: str) -> None:
        self.setLed(cmd)

    def sendCmd(self, cmd: str) -> None:
        self.port.write('{}\n'.format(cmd).encode())

    def readTo(self, tag: str, timeout: float = 1) -> bool:
        try:
            line, last = self.getResponse(timeout)
        except Exception:  # TODO what exceptions do we expect?
            return False

        if line == '':
            return False

        while line.split(',')[0] != tag:
            try:
                line, last = self.getResponse(timeout)
                if line == '':
                    return False
            except Exception:  # TODO what exceptions do we expect
                return False

        return True

    def read(self):
        """
        Read data from the serial port continuously.

        buffers each line in a list (self.comsData)
        when an end of response (^Z) is read, adds the complete list of response lines to
        self.responseData and resets the comsData list for the next command response.
        """
        comsData = []
        line = ''

        while not self.shutdown_requested.is_set():
            # read from serial 1 char at a time so we can parse each character
            val = self.port.read(1)
            if not val:
                continue

            first_byte = val[0]
            # TODO abstract magic values
            if first_byte == 13:  # ignore the CRs
                pass

            elif first_byte == 26:  # ^Z (end of response)
                if len(line) > 0:
                    # add last line to response set if it is not empty
                    comsData.append(line)
                    line = ''  # clear the line buffer for the next line
                # got the end of the command response so add the full set of response data
                # as a new item in self.responseData
                with self.readLock:
                    self.responseData.append(list(comsData))

                # clear the bucket for the lines of the next command response
                comsData = []
            # NL, terminate the current line and add it to the response data list
            # (comsData) (if it is not a blank line)
            elif first_byte == 10:
                if len(line) > 0:
                    comsData.append(line)
                    line = ''  # clear the bufer for the next line
            else:
                line = line + chr(first_byte)  # add the character to the current line buffer

    def getResponse(self, timeout=1):
        """
        Read response data for a command.

        returns tuple (line,last)
        line is one complete line of text from the command response
        last = true if the line was the last line of the response data
        (indicated by a ^Z from the neato)
        returns the next line of data from the buffer.
        if the line was the last line last = true
        if no data is avaialable and we timeout returns line=''
        """
        # if we don't have any data in currentResponse, wait for more data to come in (or timeout)
        while len(self.currentResponse) == 0 and timeout > 0:
            # pop a new response data list out of self.responseData
            # (should contain all data lines returned for the last sent command)
            with self.readLock:
                if len(self.responseData) > 0:
                    self.currentResponse = self.responseData.pop(0)
                else:
                    self.currentResponse = []  # no data to get

            if len(self.currentResponse) == 0:  # nothing in the buffer so wait (or until timeout)
                self.err('Had to sleep for 10ms')
                time.sleep(0.010)
                timeout -= 0.010

        # default to nothing to return
        line = ''
        last = False

        # if currentResponse has data pop the next line
        if not len(self.currentResponse) == 0:
            line = self.currentResponse.pop(0)
            if len(self.currentResponse) == 0:
                last = True  # if this was the last line in the response set the last flag
        else:
            self.err('Time Out')  # no data so must have timedout

        return (line, last)

    def flush(self):
        while(1):
            line, last = self.getResponse(1)
            if line == '':
                return
