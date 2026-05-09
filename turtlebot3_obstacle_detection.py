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
# Led Import
import smbus
import time
import RPi.GPIO as GPIO
from gpiozero import LED

# ---------------------------------------------------
#               LED boilerplate
# ---------------------------------------------------

# Use BCM GPIO references
# instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Define GPIO to use on Pi
GPIO_TRIGECHO = 15

# Set pins as output and input
GPIO.setup(GPIO_TRIGECHO,GPIO.OUT)  # Initial state as output

# Set trigger to False (Low)
GPIO.output(GPIO_TRIGECHO, False)

# Get I2C bus
# bus = smbus.SMBus(1) # or smbus.SMBus(0)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
# i2c_address = 0x44
# bus.write_byte_data(i2c_address, 0x01, 0x05)

RED_H = 0x0C
RED_L = 0x0B

GREEN_H = 0x0A
GREEN_L = 0x09

BLUE_H = 0x0E
BLUE_L = 0x0D

def getAndUpdateColour():
    # while True:
	# Read the data from the sensor
    data = bus.read_i2c_block_data(i2c_address, 0x09, 6)

    # upshift 
    red = data[3] << 8 | data[2]
    green = data[1] << 8 | data[0]
    blue = data[5] << 8 | data[4]

    red *= 2**(-8)
    green *= 2**(-8)
    blue *= 2**(-8)

    colors = [red, green, blue]
    return colors

class Ranges:
    # Constants for scan cleanup
    LOWER_DIST = 0
    UPPER_DIST = 3.5
    STOP_DIST = 0.25
    COLLIS_THRESHOLD = 0.15
    
    def __init__(self):
        self.angles = np.array([])
        self.dists = np.array([])
        self.stop = False
        
        # Pairs contain [angle, dist]
        self.pairs = np.empty((0, 2))
        self.front = np.empty((0, 2))
        self.left = np.empty((0, 2))
        self.back = np.empty((0, 2))
        self.right = np.empty((0, 2))

        self.collis = 0
        self.in_collis = False

        self.on_red_count = 0
        self.on_red_flag = False
        self.on_red_cooldown = 0
        self.led = LED(17)
        self.led.off()
        
        

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
            # print("No scan data")
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
    def furthest_obj_ang(self):
        name, slc, avg_dist = self.max_avg()

        # We pick the angle in the middle for smoother turning
        return np.mean(slc[:, 0])
        
    def avg_dists_slices(self):
        avg_dists_slcs = {"front": np.mean(self.front[:, 1]), 
                          "left": np.mean(self.left[:, 1]), 
                          "right" : np.mean(self.right[:, 1])}
        # ("back",  self.back,  np.mean(self.back[:, 1])) 
        return avg_dists_slcs

    # Returns min dist and its angle
    def min_dist(self):
        idx = self.dists.argmin()
        return self.angles[idx], self.dists[idx]
    
    def emergency_stop(self):
        if np.min(self.front[:, 1]) < self.STOP_DIST:
            self.stop = True 
        else:
            if self.stop:
                self.prevtheta = 0.
            self.stop = False

    def count_collis(self):
        _, min_d = self.min_dist()
        if min_d < self.COLLIS_THRESHOLD:
            if not self.in_collis:
                self.collis += 1
                self.in_collis = True
        else:
            self.in_collis = False

    def toggle_led(self, colors):
        if colors[0]/colors[1] >= 1.15:
            if not self.on_red_flag and self.on_red_cooldown + 3 <= time.time():
                self.on_red_count += 1
                self.on_red_flag = True
                self.on_red_cooldown = time.time()
                self.led.on()
        else:
            self.on_red_flag = False
            self.led.off()


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
        # Pairs of data
        self.pairs = np.column_stack((self.angles, self.dists))
        # Update slices
        self.slices()
        # If front dist min is less then STOP_DIST stop
        self.emergency_stop()

        # Collisions count
        self.count_collis()

        # Led Logic
        # colors = getAndUpdateColour()
        # self.toggle_led(colors)

        # print("The colors count is:", self.on_red_count, "\n On red:", self.on_red_flag, "\n")
        # print(colors)
        # print(self.led.value)

        

class Navigation:
    def __init__(self):
        # Speed limits
        self.vel_max = 0.22
        self.ang_vel_max = 1.60
        # Current velocity and angular velocity
        self.vel = 0.
        self.ang_vel = 0.

        # Tuning Gains
        self.k_side = 1.3   # Gain for centering in hallways
        self.k_front = 0.3  # Gain for avoiding front obstacles
        self.k_stop = 0.8   # Lower gain for turning in place

        # Front threshold
        self.front_thres = 0.6

        # Randomness added every 10 count
        self.random_count = 0
        self.random_stay_random = 2
        self.random_coeff = 0.

    def angular_velocity(self, avg_dists_slcs:dict, stop):
        # 1. Handle the "Stop" or "Trapped" case
        sides_diff = avg_dists_slcs["left"] - avg_dists_slcs["right"]
        
        if stop:
            print("Stopped\n")
            # If we are forced to stop, turn in place toward the most open space
            if np.abs(sides_diff) < 0.375:
                self.ang_vel = 0.3
            else:
                raw_ang_vel =  sides_diff * self.k_stop 
                self.ang_vel = np.clip(raw_ang_vel, -1., 1.)
            return self.ang_vel
        
        self.random_count += 1
        # 2. If not stopped we add randomness every 10 count
        if self.random_count % 10 == 0:
            print("Randomness on")
            self.random_coeff = (2*np.random.rand() - 1)*self.ang_vel_max
            self.random_stay_random = 3

        if self.random_stay_random > 0:
            self.random_stay_random -= 1
            return self.random_coeff*self.ang_vel_max

        
        # 3. Narrow hallway calculations
        steering_contribution = sides_diff*self.k_side

        # If object is close infront we increase our turn
        if avg_dists_slcs["front"] < self.front_thres:
            front_turning_factor = 1 / (avg_dists_slcs["front"] + 0.05) 
            steering_contribution += np.sign(sides_diff) * front_turning_factor * self.k_front

        # 4. Deadband
        if np.abs(steering_contribution) < 0.1:
            steering_contribution = 0.

        # 5. Clamp to limits
        self.ang_vel = np.clip(steering_contribution, -self.ang_vel_max, self.ang_vel_max)

        return self.ang_vel
    
    def velocity(self, stop):
        if stop:
            self.vel = 0.
            return self.vel
        # v needs to stay in proportion to 1/w
        # self.vel = self.vel_max / (1 + abs(self.ang_vel))

        self.vel = self.vel_max * (1.0 - (abs(self.ang_vel) / self.ang_vel_max))
        return max(self.vel, 0.05)


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')

        self.scan_ranges = Ranges()
        self.has_scan_received = False

        self.nav = Navigation()

        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.2
        self.tele_twist.angular.z = 0.0
        # Logging velocities for avg velocity data
        self.velocities = 0.
        self.vels_count = 0
        # Added randomness every 10 count
        self.random_count = 0

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
            pass
            # self.scan_ranges.ascii_map()

        avg_dists_slcs = self.scan_ranges.avg_dists_slices()
        move_stop = self.scan_ranges.stop
        # theta = self.scan_ranges.steer_dir()
        self.tele_twist.angular.z = self.nav.angular_velocity(avg_dists_slcs, move_stop)
        self.tele_twist.linear.x = self.nav.velocity(move_stop)

        self.velocities += self.tele_twist.linear.x
        self.vels_count += 1
        avg_velocity = self.velocities / self.vels_count
        print(f"v={self.tele_twist.linear.x:.4f}, w={self.tele_twist.angular.z:.3f}, \
              collisions={self.scan_ranges.collis:.4f}, avg velocity = {avg_velocity:.4f} ")

        # self.tele_twist.linear.x = 0.
        # self.tele_twist.angular.z = 0.
        self.cmd_vel_pub.publish(self.tele_twist)
       


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
