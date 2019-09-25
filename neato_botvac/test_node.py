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
import time

from neato_botvac.neato_driver import Botvac


def main():
    b = Botvac('/dev/ttyACM0')
    last_time = time.time()
    scan_ranges = None
    diffs = []

    while 1:
        b.requestScan()
        scan_ranges = b.getScanRanges()
        assert scan_ranges
        now = time.time()
        diff = now - last_time
        last_time = now
        print(diff)
        diffs.append(diff)

    print(diffs)


if __name__ == '__main__':
    main()
