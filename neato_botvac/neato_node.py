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
from math import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .neato_driver import Botvac


class NeatoNode(Node):
    """ROS2 interface to the Neato Botvac."""

    def __init__(self):
        super(NeatoNode, self).__init__('neato_botvac')
        self.bot = Botvac('/dev/ttyACM0')
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        # self.odom_pub
        # self.button_pub
        # self.sensor_pub
        # self.cmd_sub
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = 'scan'
        self.scan_msg.angle_min = 0.0
        self.scan_msg.angle_max = 359.0 * pi / 180.0
        self.scan_msg.angle_increment = 1.0 * pi / 180.0
        self.scan_msg.time_increment = 0.2
        self.scan_msg.range_min = 0.0
        self.scan_msg.range_max = 5.0

        self.scan_timer = self.create_timer(0.2, self.get_scan)

    def get_scan(self):
        self.bot.requestScan()
        self.scan_msg.ranges = self.bot.getScanRanges()
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(self.scan_msg)


def main():
    rclpy.init()
    node = NeatoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
