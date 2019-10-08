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

import pathlib

from launch import LaunchDescription
import launch.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    parameters_file_dir = pathlib.Path(__file__).resolve().parent.parent
    parameters_file_path = parameters_file_dir / 'config' / 'teleop_xbox.config.yaml'

    print(parameters_file_path)

    joy_node = Node(
        package='joy',
        node_executable='joy_node',
        node_name='joy_driver',
        output='screen',
    )
    joy_interpreter = Node(
        package='teleop_twist_joy',
        node_executable='teleop_node',
        node_name='joy_interpreter',
        parameters=[parameters_file_path],
        output='screen',
        on_exit=launch.actions.Shutdown(),
    )
    base_node = Node(
        package='neato_botvac',
        node_executable='neato',
        node_name='neato_base',
        output='screen',
    )

    return LaunchDescription([
        base_node,
        joy_node,
        joy_interpreter,
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/cartographer.launch.py']),
        #     launch_arguments={
        #         'use_sim_time': 'false',
        #     }.items(),
        # ),
    ])
