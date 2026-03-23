#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
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
#
# Authors: Jeonggeun Lim, Ryan Shim, Gilbert
from numpy import zeros, cos, sin, pi
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

class DistanceReading():
    def __init__(self, distance: float, angle_degrees: int):
        if distance <= 0 or distance >= 3.5:
            self.legit = False
        else:
            self.legit = True
        self.distance = distance
        self.angle_degrees = angle_degrees



class PointMap():
    def __init__(self, x:int, y:int):
        self._x = x
        self._y = y
        self.cleared = zeros([x, y])
        self.points = zeros([x, y])

    def _map_xy_to_matrix(self, x:float, y:float) -> tuple:
        x = int(x + self._x/2)
        y = int(y + self._y/2)
        return (x, y)

    def _store_xy(self, x:int, y:int, value:int):
        x, y = self._map_xy_to_matrix(x, y)
        out_of_bounds = False

        #normalize
        if x < 0: x = 0; out_of_bounds = True 
        if y < 0: y = 0; out_of_bounds = True
        if x >= self._x: x = self._x-1; out_of_bounds = True
        if y >= self._y: y = self._y-1; out_of_bounds = True

        self.points[x, y] = 1 if not out_of_bounds else 2

    def clear(self):
        self.points = self.cleared.copy()

    def update(self, reading:DistanceReading):
        angle = reading.angle_degrees * pi/180
        meters_to_centimeters = 100
        x = sin(angle) * reading.distance * meters_to_centimeters
        y = cos(angle) * reading.distance * meters_to_centimeters
        self._store_xy(x, y, 1)

    def print_point_map(self):
        points_to_fig = {0: ".", 1: "1", 2: "2"}
        print("_"*self._x)
        for column in range(self._x-1):
            print("|", end="")
            for row in range(self._y-1):
                print(points_to_fig[self.points[row, column]], end="")
            print("|")

class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')
        print('----------------------------------------------')
        print('stop angle: -90 ~ 90 deg')
        print('stop distance: 0.5 m')
        print('----------------------------------------------')

        self.scan_ranges = []
        self.has_scan_received = False

        self.stop_distance = 0.5
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0
        self.points = PointMap(75, 25)

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        #TODO: Print number of 0's to indicate quality of data.
        # self.scan_ranges = msg.ranges
        self.scan_ranges = [DistanceReading(reading, angle) for angle, reading in enumerate(msg.ranges)]
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        if self.has_scan_received:
            self.detect_obstacle()

    def detect_obstacle(self):
        for reading in self.scan_ranges:
            self.points.update(reading)
        self.points.print_point_map()
        self.points.clear()
        # f_range         = self.scan_ranges[0:30] + self.scan_ranges[-30:]
        # fleft_range     = self.scan_ranges[30:90]
        # fright_range    = self.scan_ranges[-90:-30]
        # b_range         = self.scan_ranges[150:-150]
        # bright_range    = self.scan_ranges[90:150]
        # bleft_range     = self.scan_ranges[-150:-90]
         
        # ranges = [f_range, fleft_range, bleft_range, b_range, bright_range, fright_range] 

        # obstacle_distance = min([min(i) for i in ranges])

        # twist = Twist()
        # if obstacle_distance < self.stop_distance:
        #     twist.linear.x = 0.0
        #     twist.angular.z = self.tele_twist.angular.z
        #     self.get_logger().info('Obstacle detected! Stopping.', throttle_duration_sec=2)
        # else:
        #     twist = self.tele_twist

        # self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
