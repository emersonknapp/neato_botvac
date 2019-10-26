#!/usr/bin/env python3
import argparse
from queue import Queue
import sys
import time
import traceback

import serial
from serial.threaded import LineReader, ReaderThread

"""
Testing app to play with ptys in the interest of making a fake robot base for simulation.

socat -d -d pty,raw,echo=0 pty,raw,echo=0
# this makes 2 ptys and prints their names
"""


class MyReader(LineReader):
    def __init__(self, q, *args, **kwargs):
        super(MyReader, self).__init__(*args, **kwargs)
        self.q = q

    def connection_made(self, transport):
        super(MyReader, self).connection_made(transport)
        print('port opened')

    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        print('port closed')

    def handle_line(self, data):
        print('received - {}'.format(data))
        self.q.put(data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('devpath')
    parser.add_argument('-e', '--echoer', action='store_true')
    args = parser.parse_args()

    q = Queue()

    def get_reader():
        return MyReader(q)

    port = serial.Serial(args.devpath, 115200, timeout=1)
    print('opened pty {}'.format(args.devpath))
    reader = ReaderThread(port, get_reader)
    reader.start()
    reader._connection_made.wait()
    protocol = reader.protocol

    if args.echoer:
        while True:
            line = q.get()
            protocol.write_line('thanks for {}'.format(line))
    else:
        num = 1
        while True:
            time.sleep(1)
            protocol.write_line('msg {}'.format(num))
            num += 1


if __name__ == '__main__':
    main()
