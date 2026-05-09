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
# import RPi.GPIO as GPIO
from gpiozero import LED

LASER_DISTANCE_UPPER = 3.5
LASER_DISTANCE_LOWER = 0.15

class LaserData:
    def __init__(self, angle, distance):
        self.angle = angle

        if distance < LASER_DISTANCE_LOWER or distance > LASER_DISTANCE_UPPER:
            self.distance = 3.5
        else:
            self.distance = distance

    def get_angle(self):
        return self.angle
    
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
        front_offset = len_readings // 4
        front_offset_cone = (len_readings*3) // 40
        front_semi_first = []
        front_semi_second = []
        front_cone_first = []
        front_cone_second = []

        for i in range(max(front_offset, front_offset_cone)):
            if i < front_offset:
                front_semi_first.append(self.scan_readings[i])
                front_semi_second.append(self.scan_readings[3*front_offset + i - 1])
            if i < front_offset_cone:
                front_cone_first.append(self.scan_readings[i])
                front_cone_second.append(self.scan_readings[len_readings - front_offset_cone + i - 1])

        self.front_semicircle = front_semi_first + front_semi_second
        self.front_cone = front_cone_first + front_cone_second

TURN_RIGHT = 1
TURN_LEFT  = 2
STOP_DISTANCE = 0.23
WALL_DISTANCE = 0.2
CHANNEL_IGNORE_DISTANCE = 0.5
MAX_LINEAR_SPEED = 0.21
MAX_ANGULAR_SPEED = 1.7
RATIO_POWER = 2


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

        self.led = LED(17)
        self.led.off()
 
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
        turn_ratio = np.add(np.multiply(np.divide(1, STOP_DISTANCE - CHANNEL_IGNORE_DISTANCE), np.subtract(d, STOP_DISTANCE)), 1) # lerp
        print("Turn ratio:", turn_ratio)
        if self.closest_obstacle_in_path.get_angle() < DEG_90:
            # obstacle is to the left
            self.set_angular_speed_vs_linear_speed(turn_ratio, TURN_RIGHT)
        elif self.closest_obstacle_in_path.get_angle() > DEG_270:
            # obstacle is to the right
            self.set_angular_speed_vs_linear_speed(turn_ratio, TURN_LEFT)
        else:
            print("Weirdness.")

    def update(self):
        if self.has_scan_received:
            print("----")
            self.find_obstacle_ahead()
            if self.should_stop():
                print("Stopping.")
                self.led.off()
                self.set_angular_speed_vs_linear_speed(1., TURN_LEFT)
            elif self.closest_obstacle_in_path != None:
                # deviate slightly
                print("Deviating slightly.")
                self.avoid_obstacle()
                self.led.on()
            else:
                # continue forward
                self.led.off()
                print("Continuing.")
                self.set_angular_speed_vs_linear_speed(0.)
            
            self.cmd_vel_pub.publish(self.tele_twist)


def main(args=None):
    try:
        rclpy.init(args=args)
        turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
        rclpy.spin(turtlebot3_obstacle_detection)
    
    except KeyboardInterrupt:
        print("Stopping navigation.")
        turtlebot3_obstacle_detection.tele_twist.linear.x = 0.0
        turtlebot3_obstacle_detection.tele_twist.angular.z = 0.0
        turtlebot3_obstacle_detection.cmd_vel_pub(turtlebot3_obstacle_detection.tele_twist)
        raise

    finally:
        turtlebot3_obstacle_detection.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()