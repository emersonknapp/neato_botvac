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
from neato_botvac.neato_driver import Accelerometer
from neato_botvac.neato_driver import BatteryStatus
from neato_botvac.neato_driver import BotvacDriver
from neato_botvac.neato_driver import BotvacDriverCallbacks
from neato_botvac.neato_driver import Encoders
from neato_botvac.neato_driver import Scan


def test_constructors():
    callbacks = BotvacDriverCallbacks(
        encoders=lambda x: None,
        battery=lambda x: None,
        scan=lambda x: None,
        accel=lambda x: None,
    )
    driver = BotvacDriver(callbacks=callbacks)
    assert driver

    enc = Encoders.from_state(0, {
        'LeftWheel_PositionInMM': '12',
        'RightWheel_PositionInMM': '20',
    })
    assert enc

    bat = BatteryStatus.from_state(0, {
        'VBattV': '1.5',
        'BattTempCAvg': '12.32',
        'Discharge_mAH': '123.5',
        'FuelPercent': '99.3',
    })
    assert bat

    scan = Scan(0, [1, 2, 3, 4, 5])
    assert scan

    accel = Accelerometer.from_state(0, {
        'PitchInDegrees': '12.5',
        'RollInDegrees': '45.1',
        'XInG': '0.112',
        'YInG': '0.767',
        'ZInG': '0.354',
        'SumInG': '0.999',
    })
    assert accel
