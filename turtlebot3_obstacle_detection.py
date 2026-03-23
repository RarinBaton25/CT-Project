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

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan


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
        self.tele_twist.angular.z = 0.5

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
        # self.scan_ranges = msg.ranges
        self.scan_ranges = [3.5 if i>=3.5 or i<=0 else i for i in msg.ranges]
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        if self.has_scan_received:
            self.detect_obstacle()

    def detect_obstacle(self):

        # Scan ranges
        f_range = self.scan_ranges[0:30] + self.scan_ranges[-30:]
        fleft_range = self.scan_ranges[30:90]
        fright_range = self.scan_ranges[-90:-30]
        b_range = self.scan_ranges[150:-150]
        bright_range = self.scan_ranges[90:150]
        bleft_range = self.scan_ranges[-150:-90]
        
        ranges = [f_range, fleft_range, fright_range, b_range, bleft_range, bright_range] 

        front_dist = min(ranges[0:3])
        back_dist = min(ranges[3:])

        twist = Twist()
        # if obstacle_distance < self.stop_distance:
        #     twist.linear.x = 0.0
        #     twist.angular.z = self.tele_twist.angular.z
        #     self.get_logger().info('Obstacle detected! Stopping.', throttle_duration_sec=2)
        # else:
        #     twist = self.tele_twist

        def turn(self, x, dir):
            if dir == "left":
                return x*self.tele_twist, self.tele_twist.angular.z
            elif dir == "right":
                return x*self.tele_twist, self.tele_twist.angular.z
            else:
                return x*self.tele_twist, 0

        if front_dist < back_dist: # Turn forward
            if fleft_range > fright_range:
                twist.linear.x, twist.angular.z = turn_dir(1, "left")
            else:
                twist.angular.z = turn_dir(1, "right")
        elif front_dist > back_dist: # Turn backwards
            turn(1, 0)
            

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
