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
import signal

# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# from tf_msgs.msg import TFMessage

from .neato_driver import Botvac


def deg_to_rad(deg):
    return deg * pi / 180.0


class NeatoNode(Node):
    """ROS2 interface to the Neato Botvac."""

    def __init__(self):
        super(NeatoNode, self).__init__('neato_botvac')
        self.bot = Botvac('/dev/ttyACM0')
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        # self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # self.tf_pub = self.create_publisher(TFMessage, 'tf', 10)
        # self.button_pub
        # self.sensor_pub
        self.cmd_vel = (0, 0)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.scan_msg = LaserScan(
            angle_min=0.0,
            angle_max=deg_to_rad(359),
            angle_increment=deg_to_rad(1.0),
            time_increment=0.2,
            range_min=0.2,
            range_max=5.0,
        )
        self.scan_msg.header.frame_id = 'scan'

        self.scan_timer = self.create_timer(0.2, self.update)

    def update(self):
        left, right = self.bot.getMotors()
        velx, vely = self.cmd_vel
        self.bot.setMotors(velx, vely, max(abs(velx), abs(vely)))

        self.bot.requestScan()
        self.scan_msg.ranges = self.bot.getScanRanges()
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(self.scan_msg)

    def cmd_vel_cb(self, msg: Twist):
        # TODO move control knowledge to the driver?
        x = msg.linear.x * 1000
        theta = msg.angular.z * self.bot.base_width / 2
        k = max(abs(x - theta), abs(x + theta))
        if k > self.bot.max_speed:
            x = x * self.robot.max_speed / k
            theta = theta * self.robot.max_speed / k
        self.cmd_vel = (int(x - theta), int(x + theta))
        self.get_logger().info('Cmd vel ({},{})'.format(self.cmd_vel[0], self.cmd_vel[1]))

    def shutdown(self, *args):
        self.scan_timer.cancel()
        self.bot.shutdown()


def main():
    rclpy.init()
    node = NeatoNode()
    # TODO this shutdown is not working correctly, we're getting caught up in get_scan
    signal.signal(
        signal.SIGINT,
        lambda unused_signal, unused_frame: node.shutdown()
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
