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
# from numpy import zeros, cos, sin, pi
import numpy as np
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan

class Ranges:
    # Constants for scan cleanup
    LOWER_DIST = 0
    UPPER_DIST = 3.5
    STOP_DIST = 0.20
    COLLIS_THRESHOLD = 0.15
    
    def __init__(self):
        self.angles = np.array([])
        self.dists = np.array([])
        self.stop = False
        self.prevtheta = 0
        # Pairs contain [angle, dist]
        self.pairs = np.empty((0, 2))
        self.front = np.empty((0, 2))
        self.left = np.empty((0, 2))
        self.back = np.empty((0, 2))
        self.right = np.empty((0, 2))

        self.collis = 0
        self.in_collis = False
        

    # def debug_print(self):
    #     def stats(name, arr):
    #         if len(arr) == 0:
    #             return f"{name:>5}: empty"
    #         return (
    #             f"{name:>5}: "
    #             f"mean={np.mean(arr[:,1]):.2f}  "
    #             f"min={np.min(arr[:,1]):.2f}  "
    #             f"max={np.max(arr[:,1]):.2f}  "
    #             f"theta=[{arr[0,0]:.2f}, {arr[-1,0]:.2f}]"
    #         )

    #     print("\n--- scan map ---")
    #     print(stats("front", self.front))
    #     print(stats("left",  self.left))
    #     print(stats("back",  self.back))
    #     print(stats("right", self.right))

    #     theta_min, dist_min = self.min_dist()
    #     print(f"closest: dist={dist_min:.2f}, theta={theta_min:.2f}")

    def ascii_map(self, size=21):
        """
        Print a top-down ASCII LiDAR map.
        Robot is at center: R
        Obstacles are marked with: #
        Empty cells are: .
        """
        if len(self.pairs) == 0:
            print("No scan data")
            return

        grid = np.full((size, size), '.', dtype='<U1')
        c = size // 2
        grid[c, c] = 'R'

        max_range = self.UPPER_DIST

        for theta, dist in self.pairs:
            if not np.isfinite(dist):
                continue

            # Polar -> Cartesian
            x = dist * np.cos(theta)
            y = dist * np.sin(theta)

            # Scale into grid coordinates
            gx = int(round((x / max_range) * (c - 1)))
            gy = int(round((y / max_range) * (c - 1)))

            row = c - gy
            col = c + gx

            if 0 <= row < size and 0 <= col < size and grid[row, col] == '.':
                grid[row, col] = '#'

        print("\n--- LiDAR map ---")
        for row in grid:
            print(" ".join(row))

    # Returns maximum avg dist
    def max_avg(self):
        candidates = [
        ("front", self.front, np.mean(self.front[:, 1])),
        ("left",  self.left,  np.mean(self.left[:, 1])),
        # ("back",  self.back,  np.mean(self.back[:, 1])),
        ("right", self.right, np.mean(self.right[:, 1]))
    ]
        name, slc, avg_dist = max(candidates, key=lambda x: x[2])
        return name, slc, avg_dist
    
    # Angle for obj furthest away
    def max_theta(self):
        name, slc, avg_dist = self.max_avg()
        # Largest val in slc
        # idx = slc[:, 1].argmax()
        # theta = slc[idx, 0]
        # return theta

        # We pick the angle in the middle for smoother turning
        return np.mean(slc[:, 0])
        
        

    # Direction to steer in using prev angle for smoother steering
    def steer_dir(self):
        prev_weight = 0.8
        new_weight = 0.2
        theta = prev_weight*self.prevtheta + new_weight*self.max_theta()
        theta = self.wrap_theta(theta)
        # Store prev value
        self.prevtheta = theta
        return theta
    
    def wrap_theta(self, theta):
        return (theta + np.pi) % (2*np.pi) - np.pi


    # Returns min dist and its angle
    def min_dist(self):
        idx = self.dists.argmin()
        return self.angles[idx], self.dists[idx]
    
    def slices(self):
        # Offsets
        n = len(self.pairs)
        w = n // 8
        h = n // 2
        q = n // 4
        q2 = 3 * n // 4
        # Slice arrays
        self.front = np.vstack((self.pairs[:w], self.pairs[-w:]))
        self.left = self.pairs[q - w : q + w]
        self.back = self.pairs[h - w : h + w]
        self.right = self.pairs[q2 - w : q2 + w]

    def update(self, msg):
        # Clean up data and insert into dists
        self.dists = np.array([r if self.LOWER_DIST < r < self.UPPER_DIST 
                             else self.UPPER_DIST for r in msg.ranges])
        # Array of the angles from ranges
        self.angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        # Pairs data
        self.pairs = np.column_stack((self.angles, self.dists))
        # Update slices
        self.slices()
        # If front dist min is less then STOP_DIST stop
        if np.min(self.front[:, 1]) < self.STOP_DIST:
            self.stop = True 
        else:
            self.stop = False

        # Collisions count
        _, min_d = self.min_dist()
        if min_d < self.COLLIS_THRESHOLD:
            if not self.in_collis:
                self.collis += 1
                self.in_collis = True
        else:
            self.in_collis = False

        

class Navigation:
    def __init__(self):
        self.v_max = 0.4
        self.w_max = 0.67
        self.v = 0
        self.w = 0
        # Tuning constant
        self.k = .5
    def angular_velocity(self, theta):
        if(abs(theta) < 0.15):
          self.w = 0.
        else: 
            self.w = self.k*theta
        # We clamp the angular velocity since the angle can grow large
        self.w = max(-self.w_max, min(self.w, self.w_max))
        return self.w

    def velocity(self):
        # v needs to stay in proportion to 1/w
        self.v = self.v_max / (1 + abs(self.w))
        return self.v


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')
        print('----------------------------------------------')
        print('stop angle: -90 ~ 90 deg')
        print('stop distance: 0.5 m')
        print('----------------------------------------------')

        self.scan_ranges = Ranges()
        self.has_scan_received = False

        # self.stop_distance = 0.5
        self.nav = Navigation()
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0

        self.debug_counter = 0

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


    # Writes data from scan
    def scan_callback(self, msg):
        self.scan_ranges.update(msg)
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        if self.has_scan_received:
            self.detect_obstacle()

    
    def detect_obstacle(self):
        # self.scan_ranges.debug_print()

        self.debug_counter += 1
        if self.debug_counter % 10 == 0:
            self.scan_ranges.ascii_map()

        theta = self.scan_ranges.steer_dir()
        self.tele_twist.angular.z = self.nav.angular_velocity(theta)

        if self.scan_ranges.stop:
            self.tele_twist.linear.x = 0.
            self.tele_twist.angular.z += np.sign(theta)*0.67
        else:
            self.tele_twist.linear.x = self.nav.velocity()

        print(f"theta={theta:.2f}, v={self.tele_twist.linear.x:.2f}, w={self.tele_twist.angular.z:.2f}, collisions={self.scan_ranges.collis:.2f}")

        self.cmd_vel_pub.publish(self.tele_twist)
       


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
