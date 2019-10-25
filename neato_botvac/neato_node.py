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
from math import cos, pi, sin
import threading

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

from .neato_driver2 import (
    BotvacDriver,
    BotvacDriverCallbacks,
)


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

    def __init__(self, shutdown_signal: threading.Event):
        super(NeatoNode, self).__init__('neato_botvac')
        # self.bot = Botvac(shutdown_signal, '/dev/ttyACM0')
        self.bot = BotvacDriver('/dev/ttyACM0', callbacks=BotvacDriverCallbacks(
            encoders=self.encoders_cb,
            battery=self.battery_cb,
            scan=self.scan_cb))
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
        self.odom_to_base_link_tf = TransformStamped(
            header=Header(frame_id='odom'),
            child_frame_id='base_link')
        self.base_link_to_scan_tf = TransformStamped(
            header=Header(frame_id='base_link'),
            child_frame_id='scan',
            transform=Transform(
                translation=Vector3(x=0., y=0., z=0.),
                rotation=Quaternion(x=0., y=0., z=0., w=1.),
            ))
        self.odometer = Odometer(self.get_clock(), self.bot.base_width)

        self.tf_timer = self.create_timer(0.025, self.pub_tf)
        self.scan_timer = self.create_timer(0.2, self.bot.requestScan)
        self.battery_timer = self.create_timer(1, self.bot.requestBattery)
        self.encoder_timer = self.create_timer(0.1, self.bot.requestEncoders)

    def pub_tf(self):
        now = self.get_clock().now().to_msg()
        self.odom_to_base_link_tf.header.stamp = now
        self.odom_to_base_link_tf.transform = self.odometer.transform_msg
        self.base_link_to_scan_tf.header.stamp = now
        self.tf_pub.publish(TFMessage(transforms=[
            self.odom_to_base_link_tf,
            self.base_link_to_scan_tf,
        ]))

    def scan_cb(self, scan_data):
        self.scan_msg.ranges = scan_data.ranges
        # TODO use the stamp from the data
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(self.scan_msg)

    def encoders_cb(self, encoders_data):
        self.odometer.update(encoders_data.left, encoders_data.right)
        # TODO use the stamp from the data
        self.odom_pub.publish(self.odometer.odom_msg)

    def battery_cb(self, battery_data):
        self.battery_msg.voltage = battery_data.voltage
        self.battery_msg.temperature = battery_data.temperature
        self.battery_msg.current = battery_data.current
        self.battery_msg.percentage = battery_data.percentage
        self.battery_msg.present = True
        self.battery_msg.header.stamp = self.get_clock().now().to_msg()
        # TODO use the stamp from the data
        self.battery_pub.publish(self.battery_msg)

    def update(self):
        left, right = self.bot.getMotors()
        velx, vely = self.cmd_vel
        # TODO send cmd_vels
        self.bot.setMotors(velx, vely, max(abs(velx), abs(vely)))

    def cmd_vel_cb(self, msg: Twist):
        # TODO move control knowledge to the driver?
        x = msg.linear.x * 1000
        theta = msg.angular.z * self.bot.base_width / 2
        k = max(abs(x - theta), abs(x + theta))
        if k > self.bot.max_speed:
            x = x * self.bot.max_speed / k
            theta = theta * self.bot.max_speed / k
        self.cmd_vel = (int(x - theta), int(x + theta))


def main():
    rclpy.init()
    shutter = threading.Event()
    node = NeatoNode(shutter)
    rclpy.spin(node)
    shutter.set()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
