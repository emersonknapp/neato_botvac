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
from ament_copyright.main import main as copyright_main
from ament_flake8.main import main as flake8_main
# from ament_mypy.main import main as mypy_main
from ament_pep257.main import main as pep257_main
import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    assert copyright_main(argv=['.']) == 0, 'Found errors'


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    assert flake8_main(argv=[]) == 0, 'Found errors'


# @pytest.mark.mypy
# @pytest.mark.linter
# def test_mypy():
#     assert mypy_main(argv=[]) == 0, 'Found errors'


@pytest.mark.pep257
@pytest.mark.linter
def test_pep257():
    assert pep257_main(argv=[]) == 0, 'Found errors'
