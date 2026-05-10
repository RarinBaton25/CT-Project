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
import RPi.GPIO as GPIO
from gpiozero import LED
# Led Import
import smbus
import time
import colorsys
# import RPi.GPIO as GPIO
from gpiozero import LED

# ---------------------------------------------------
#               LED boilerplate
# ---------------------------------------------------

# Get I2C bus
bus = smbus.SMBus(1) # or smbus.SMBus(0)

# ISL29125 address, 0x44(68)
# Select configuation-1register, 0x01(01)
# 0x0D(13) Operation: RGB, Range: 360 lux, Res: 16 Bits
i2c_address = 0x44
bus.write_byte_data(i2c_address, 0x01, 0x05)

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

# ---------------------------------------------------

LASER_DISTANCE_UPPER = 3.5
LASER_DISTANCE_LOWER = 0.15

class LaserData:
    def __init__(self, angle, distance):
        self.angle = angle
        self.real_distance = distance

        if distance < LASER_DISTANCE_LOWER or distance > LASER_DISTANCE_UPPER:
            self.distance = 3.5
        else:
            self.distance = distance

    def get_angle(self):
        return self.angle
    
    def get_real_distance(self):
        return self.real_distance
    
    def get_distance(self):
        return self.distance

class LaserReading:
    def __init__(self):
        self.scan_readings = []
        self.front_semicircle:list[LaserData] = []
        self.front_cone:list[LaserData] = []

    def update_readings(self, msg):
        self.scan_readings = [LaserData(data_index*msg.angle_increment + msg.angle_min, distance) \
                            for data_index, distance in enumerate(msg.ranges)]

        # Offsets
        len_readings = len(self.scan_readings)
        front_offset = len_readings // 5
        front_offset_cone = (len_readings*3) // 40
        front_semi_first = []
        front_semi_second = []
        front_cone_first = []
        front_cone_second = []

        for i in range(max(front_offset, front_offset_cone)):
            if i < front_offset:
                front_semi_first.append(self.scan_readings[i])
                front_semi_second.append(self.scan_readings[len_readings - front_offset + i - 1])
            if i < front_offset_cone:
                front_cone_first.append(self.scan_readings[i])
                front_cone_second.append(self.scan_readings[len_readings - front_offset_cone + i - 1])

        self.front_semicircle = front_semi_first + front_semi_second
        self.front_cone = front_cone_first + front_cone_second

TURN_RIGHT = 1
TURN_LEFT  = 2
STOP_DISTANCE = 0.19
WALL_DISTANCE = 0.2
CHANNEL_IGNORE_DISTANCE = 0.5
# Debug = 0 for on, Debug = 1 for off
DEBUG = 1
MAX_LINEAR_SPEED = 0.21*DEBUG
MAX_ANGULAR_SPEED = 1.7*DEBUG
RATIO_POWER = 1.4
MAX_TURNING_RATIO_FOR_DEVIATION = 1.

# together, these make the front hemisphere
DEG_90  = np.multiply(0.5, np.pi)
DEG_270 = np.multiply(1.5, np.pi)

class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')

        self.turn_ratio = 0.

        self.scan_ranges = LaserReading()
        self.has_scan_received = False

        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.0
        self.tele_twist.angular.z = 0.0

        self.closest_obstacle_in_path:LaserData = None

        # Navigation LED
        self.navigation_led = LED(17)
        self.navigation_led.off()

        # Victim detection
        self.victim_count = 0
        self.on_red_flag = False
        self.on_red_cooldown = 0
        self.victim_led = LED(27)
        self.victim_led.off()

        # Average speed variables
        self.speed_sum = 0.
        self.speed_sample_count = 0

        # Collission variables
        self.collission_count = 0
 
        # Data for collisions
        self.branches = None
        self.branches_avg = None
        self.branches_count = 0

        # Led stats
        self.h_sum = 0.
        self.s_sum = 0.
        self.v_sum = 0.
        self.led_count = 0

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

        self.timer = self.create_timer(0.05, self.timer_callback)

    # Writes data from scan
    def scan_callback(self, msg):
        self.scan_ranges.update_readings(msg)
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
        self.update()

    def should_stop(self):
        for data in self.scan_ranges.front_cone:
            if data.get_distance() < STOP_DISTANCE:
                return True
        return False
    def set_angular_speed_vs_linear_speed(self, ratio:float, turn_direction=TURN_LEFT):
        """
        Assigns angular and linear speed values. They must still be published.
        The ratio determines the relation between turning and driving forwards.
        0. is straight ahead, 1. is turning on the spot. Values inbetween are interpolated.
        A turning direction can be passed, though turning left is assumed.
        """
        if not 0. <= ratio <= 1.:
            print("Ratio is out of bounds:", ratio)
            ratio = 1.

        ratio = ratio**RATIO_POWER

        self.tele_twist.linear.x  = (1. - ratio)*MAX_LINEAR_SPEED
        print("Setting linear.x =", self.tele_twist.linear.x)

        if turn_direction == TURN_LEFT:
            self.tele_twist.angular.z =  ratio*MAX_ANGULAR_SPEED
        elif turn_direction == TURN_RIGHT:
            self.tele_twist.angular.z = -ratio*MAX_ANGULAR_SPEED
        else:
            print("Invalid turn direction provided.")
            self.tele_twist.angular.z = 0.

        print("Setting angular.z =", self.tele_twist.angular.z)

    def distance_to_channel_wall(self, angle):
        """
        Calculates the distance to walls defined by 1/sin(angle)
        """
        if angle == 0.:
            return 100 # something big
        return np.multiply(np.abs(np.divide(1, np.sin(angle))), WALL_DISTANCE)
        
    def find_obstacle_ahead(self):
        """
        Finds closest obstacle in the path of the robot. Stores the closest obstacle in self.closest_obstacle_in_path, else None
        """
        # print( [[data.get_angle(), f"{data.get_distance()} < {self.distance_to_channel_wall(data.get_angle())} < 3.5"] for data in self.scan_ranges if data.get_distance() < self.distance_to_channel_wall(data.get_angle()) < 3.5] )
        self.closest_obstacle_in_path = None
        for data in self.scan_ranges.front_semicircle:
             #limit search to obstacles in front
                if data.get_distance() < self.distance_to_channel_wall(data.get_angle()) < CHANNEL_IGNORE_DISTANCE: # is there an obstacle in our path?
                    if self.closest_obstacle_in_path == None or data.get_distance() < self.closest_obstacle_in_path.get_distance():
                        # update newest closest obstacle
                        self.closest_obstacle_in_path = data

    def avoid_obstacle(self):
        d = self.closest_obstacle_in_path.get_distance()
        turn_ratio = np.add(np.multiply(np.divide(MAX_TURNING_RATIO_FOR_DEVIATION, STOP_DISTANCE - CHANNEL_IGNORE_DISTANCE), \
                                         np.subtract(d, STOP_DISTANCE)), MAX_TURNING_RATIO_FOR_DEVIATION) # lerp
        print("Turn ratio:", turn_ratio)
        if self.closest_obstacle_in_path.get_angle() < DEG_90:
            # obstacle is to the left
            self.set_angular_speed_vs_linear_speed(turn_ratio, TURN_RIGHT)
        elif self.closest_obstacle_in_path.get_angle() > DEG_270:
            # obstacle is to the right
            self.set_angular_speed_vs_linear_speed(turn_ratio, TURN_LEFT)
        else:
            print("Weirdness.")

    def update_victim_led(self, colors:list):
        # 1. Normalize RGB
        r, g, b = [c / 255.0 for c in colors]

        # 2. Convert to HSV
        h, s, v = colorsys.rgb_to_hsv(r, g, b)

        # 3. Logic for detecting red, value bounds tuned by averaging hue, saturation and value
        #    by sampling dataS
        self.led_count += 1
        self.h_sum += h
        self.s_sum += s
        self.v_sum += v
        # Code for sampling red data
        # print("\n", h, s, v, "\n")
        # print("\n h_avg =", self.h_sum/self.led_count, "s_avg =", self.s_sum / self.led_count, "v_avg =", self.v_sum / self.led_count)

        is_red = (0.10 < h < 0.16) and (s > 0.47) and v > 0.105
        if is_red:
            if not self.on_red_flag and self.on_red_cooldown + 3 <= time.time():
                self.victim_count += 1
                self.on_red_flag = True
                self.on_red_cooldown = time.time()
                self.victim_led.on()
        else:
            self.on_red_flag = False
            self.victim_led.off()

    def collission_detected(self):
        return False

    def update(self):
        if self.has_scan_received:
            print("----")
            self.find_obstacle_ahead()
            colors = getAndUpdateColour()
            self.update_victim_led(colors)
            if self.should_stop():
                print("Stopping.")
                self.navigation_led.off()
                self.set_angular_speed_vs_linear_speed(1., TURN_LEFT)
            elif self.closest_obstacle_in_path != None:
                # deviate slightly
                print("Deviating slightly.")
                self.avoid_obstacle()
                self.navigation_led.on()
            else:
                # continue forward
                self.navigation_led.off()
                print("Continuing.")
                self.set_angular_speed_vs_linear_speed(0.)

            if self.collission_detected():
                self.collission_count += 1
            
            # Victims
            print("Victims found:", self.victim_count)
            
            # Average Speed
            self.speed_sample_count += 1
            self.speed_sum += self.tele_twist.linear.x
            print("Average speed:", np.divide(self.speed_sum, self.speed_sample_count))
            
            # Collissions
            print("Collissions detected:", self.collission_count)
            self.cmd_vel_pub.publish(self.tele_twist)

            # Prints for collisions tuning
            # self.branches_count += 1
            # if type(self.branches) == type(None):
            #     self.branches = np.array([data.get_real_distance() for data in self.scan_ranges.scan_readings])
            #     self.branches_avg = self.branches
            # else:
            #     self.branches += np.array([data.get_real_distance() for data in self.scan_ranges.scan_readings])
            #     self.branches_avg = self.branches / self.branches_count
            # print("Average for each branch at time t =", self.branches_count, "is \n branches =", self.branches_avg, "\n")


def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3ObstacleDetection()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("Stopping navigation.")
        node.tele_twist.linear.x = 0.0
        node.tele_twist.angular.z = 0.0
        node.cmd_vel_pub.publish(node.tele_twist)
        time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()