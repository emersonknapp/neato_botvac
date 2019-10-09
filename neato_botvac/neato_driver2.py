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
import asyncio
import threading
import time
import traceback
import sys

import serial
from serial.threaded import LineReader, ReaderThread


class BotvacConnectionError(Exception):
    pass


class BotvacDriver():
    def __init__(self, port: str = '/dev/ttyACM0', baud: int = 115200):
        self.port = serial.serial(port, baud, timeout=0.1)
        if not self.port.isOpen():
            raise BotvacConnectionError('Could not open connection to the botvac')
        self.info('Connection established')

        self.readThread = threading.Thread(target=self.read, daemon=True)

    def err(self, msg: str):
        print(msg)

    def info(self, msg: str):
        print(msg)


# class Output(asyncio.Protocol):
#     def connection_made(self, transport):
#         self.transport = transport
#         print('port opened', transport)
#         transport.serial.rts = False
#         transport.write(b'getaccel\n')
#
#     def data_received(self, data):
#         print('data received', repr(data))
#         self.transport.close()
#
#     def connection_lost(self, exc):
#         print('port closed')
#         asyncio.get_event_loop().stop()


# if __name__ == '__main__':
    # loop = asyncio.get_event_loop()
    # coro = serial.aio.create_serial_connection(loop, Output, '/dev/ttyUSB0', baudrate=115200)
    # loop.run_until_complete(coro)
    # loop.run_forever()
    # loop.close()


class PrintLines(LineReader):
    def __init__(self, *args, **kwargs):
        super(PrintLines, self).__init__(*args, **kwargs)
        self.current = []

    def connection_made(self, transport):
        super(PrintLines, self).connection_made(transport)
        sys.stdout.write('port opened\n')
        self.write_line('help')

    def handle_line(self, data):
        if data == '\x1a':
            print('Received full message\n')
            print(self.current)
            self.current = []
            return
        self.current.append(data)
        # sys.stdout.write('line received: {}\n'.format(repr(data)))

    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')


def main():
    ser = serial.serial_for_url('/dev/ttyACM0', baudrate=115200, timeout=1)
    with ReaderThread(ser, PrintLines) as protocol:
        while 1:
            protocol.write_line('getldsscan')
            time.sleep(0.2)


if __name__ == '__main__':
    main()
