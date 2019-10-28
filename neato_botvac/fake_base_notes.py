#!/usr/bin/env python3

"""
Notes.

socat -d -d pty,raw,echo=0 pty,raw,echo=0
# prints the names of the ptys, get from stdout
"""
import serial


def main():
    port = serial.Serial()
    pass


if __name__ == '__main__':
    main()
