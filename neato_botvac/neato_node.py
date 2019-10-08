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
from math import cos, nan, pi, sin
import signal

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from .neato_driver import Botvac


def nano_to_sec(nanos):
    return nanos / (10 ** 9)


def deg_to_rad(deg):
    return deg * pi / 180.0


class Odometer:
    def __init__(self, clock, base_width):
        self.clock = clock
        self.base_width = base_width
        self.last_time = clock.now()

        # Tracking values
        self.last_encoders = (0, 0)
        self.x = 0.
        self.y = 0.
        self.th = 0.
        self.linear = Vector3()
        self.angular = Vector3()
        self.orientation = Quaternion()

    def update(self, left, right):
        now = self.clock.now()
        dt = nano_to_sec((now - self.last_time).nanoseconds)

        d_left = (left - self.last_encoders[0]) / 1000.0
        d_right = (right - self.last_encoders[1]) / 1000.0
        self.last_encoders = (left, right)

        dx = (d_left + d_right) / 2
        dth = (d_right - d_left) / (self.base_width / 1000.0)

        x = cos(dth) * dx
        y = -sin(dth) * dx
        self.x += cos(self.th) * x - sin(self.th) * y
        self.y += sin(self.th) * x + cos(self.th) * y
        self.th += dth
        self.linear.x = dx / dt
        self.angular.z = dth / dt
        self.orientation.z = sin(self.th / 2.0)
        self.orientation.w = cos(self.th / 2.0)
        self.last_time = now

    @property
    def odom_msg(self) -> Odometry:
        return Odometry(
            header=Header(stamp=self.clock.now().to_msg(), frame_id='odom'),
            child_frame_id='base_link',
            pose=PoseWithCovariance(pose=Pose(
                position=Point(x=self.x, y=self.y, z=0.),
                orientation=self.orientation,
            )),
            twist=TwistWithCovariance(twist=Twist(
                linear=self.linear,
                angular=self.angular,
            )),
        )

    @property
    def transform_msg(self) -> Transform:
        return Transform(
            translation=Vector3(x=self.x, y=self.y, z=0.),
            rotation=self.orientation)


class NeatoNode(Node):
    """ROS2 interface to the Neato Botvac."""

    def __init__(self):
        super(NeatoNode, self).__init__('neato_botvac')
        self.bot = Botvac('/dev/ttyACM0')
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 2)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_pub = self.create_publisher(TFMessage, 'tf', 10)

        self.cmd_vel = (0, 0)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.scan_msg = LaserScan(
            angle_min=0.0,
            angle_max=deg_to_rad(359),
            angle_increment=deg_to_rad(1.0),
            time_increment=0.0,
            scan_time=0.,
            range_min=0.2,
            range_max=5.0)
        self.scan_msg.header.frame_id = 'scan'
        self.battery_msg = BatteryState()
        self.odometer = Odometer(self.get_clock(), self.bot.base_width)

        self.scan_timer = self.create_timer(0.2, self.update)
        self.tf_timer = self.create_timer(0.01, self.pub_tf)

    def pub_tf(self):
        now = self.get_clock().now().to_msg()
        odom_to_base_link_tf = TransformStamped(
            header=Header(stamp=now, frame_id='odom'),
            child_frame_id='base_link',
            transform=self.odometer.transform_msg)
        base_link_to_scan_tf = TransformStamped(
            header=Header(stamp=now, frame_id='base_link'),
            child_frame_id='scan',
            transform=Transform(
                translation=Vector3(x=0., y=0., z=0.),
                rotation=Quaternion(x=0., y=0., z=0., w=1.),
            ))
        self.tf_pub.publish(TFMessage(transforms=[
            odom_to_base_link_tf,
            base_link_to_scan_tf,
        ]))

    def update(self):
        clock = self.get_clock()
        left, right = self.bot.getMotors()
        velx, vely = self.cmd_vel
        self.bot.setMotors(velx, vely, max(abs(velx), abs(vely)))

        # Request all needed info
        self.bot.getCharger()
        self.bot.requestScan()

        # Publish messages
        self.scan_msg.ranges = self.bot.getScanRanges()
        self.scan_msg.header.stamp = clock.now().to_msg()
        self.scan_pub.publish(self.scan_msg)

        self.battery_msg.voltage = self.bot.state.get('VBattV', nan)
        self.battery_msg.temperature = self.bot.state.get('BattTempCAvg', nan)
        self.battery_msg.current = self.bot.state.get('Discharge_mAH', nan)
        self.battery_msg.percentage = self.bot.state.get('FuelPercent', nan)
        self.battery_msg.present = True
        self.battery_msg.header.stamp = clock.now().to_msg()
        self.battery_pub.publish(self.battery_msg)

        self.odometer.update(left, right)
        self.odom_pub.publish(self.odometer.odom_msg)

    def cmd_vel_cb(self, msg: Twist):
        # TODO move control knowledge to the driver?
        x = msg.linear.x * 1000
        theta = msg.angular.z * self.bot.base_width / 2
        k = max(abs(x - theta), abs(x + theta))
        if k > self.bot.max_speed:
            x = x * self.bot.max_speed / k
            theta = theta * self.bot.max_speed / k
        self.cmd_vel = (int(x - theta), int(x + theta))

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
