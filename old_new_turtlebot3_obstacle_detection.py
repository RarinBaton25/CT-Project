from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import numpy as np

# mostly just clamps range measurements
# also provides getter and setter
class RangeMeasurement:
    MINIMUM_MEASURABLE_DISTANCE = 0.  # values below this are definitely faulty
    MAXIMUM_MEASURABLE_DISTANCE = 3.5 # values above this are assumed to be faulty
    RADIAN_TO_DEGREE            = 180 / np.pi

    def __init__(self, measured_angle:float, measured_distance: float):
        self.clamped  = False
        self.valid    = True
        self.angle    = measured_angle
        self.measured_distance = measured_distance

        if self.measured_distance > self.MAXIMUM_MEASURABLE_DISTANCE:
            self.distance = self.MAXIMUM_MEASURABLE_DISTANCE
            self.clamped = True
        elif self.measured_distance < self.MINIMUM_MEASURABLE_DISTANCE:
            self.distance = self.MAXIMUM_MEASURABLE_DISTANCE # if below minimum, set to maximum for the purpose of interfering the least
            self.valid = False # it's not a real measurement, because negative distances are impossible
        else:
            self.distance = self.measured_distance

    def __repr__(self):
        return f"Angle: {self.get_angle()} Distance: {self.get_distance()}"

    def is_valid(self) -> bool:
        """If a distance was negative, it's flagged as invalid
        """
        return self.valid

    def get_distance(self) -> float:
        """Returns a range measurement's distance value.
        """
        return self.distance
    
    def get_angle(self) -> float:
        """Returns a range measurement's angle.
        """
        return self.angle * self.RADIAN_TO_DEGREE

    def get_unclamped_distance(self) -> float:
        """Returns a range measurement's distance value from before it was clamped.
        """
        return self.measured_distance

class Ranges:
    def __init__(self):
        self.range_measurements = []

    def update_range_measurements(self, scan_message):
        self.range_measurements = [RangeMeasurement(angle, range_measurement) for angle, range_measurement in zip(np.arange(len(scan_message.ranges))*scan_message.angle_increment + scan_message.angle_min, scan_message.ranges)]

    def get_range_measurements_slice(self, from_angle = None, to_angle = None):
        """Get a slice of range measurements from angle 'from_angle' to angle 'to_angle'.

        'from_angle' and 'to_angle' must each be in degrees from -360 to 360.
        """
        if from_angle != None and (from_angle < -360 or from_angle > 360):
            raise Exception("'from_angle' is out of bounds. Specified angle was", from_angle)
        if to_angle != None and (to_angle < -360 or to_angle > 360):
            raise Exception("'to_angle' is out of bounds. Specified angle was", to_angle)

        # support for negative angles
        # -30 ~ 330
        from_angle_non_negative = from_angle
        if from_angle_non_negative != None and from_angle_non_negative < 0:
            from_angle_non_negative += 360
        to_angle_non_negative = to_angle
        if to_angle_non_negative != None and to_angle_non_negative < 0:
            to_angle_non_negative += 360

        if from_angle > to_angle and from_angle != None and to_angle != None: # swap them so from_angle is <= to_angle
            temp = from_angle
            from_angle = to_angle
            to_angle = temp
        
        if from_angle == None:
            if to_angle == None:
                raise Exception("No parameters for the slice were given.")
            else:
                # only to_angle specified
                return [range_measurement for range_measurement in self.range_measurements if range_measurement.get_angle() < to_angle_non_negative]
        else:
            # from_angle specified
            if to_angle == None:
                # only from_angle specified
                return [range_measurement for range_measurement in self.range_measurements if from_angle_non_negative <= range_measurement.get_angle()]
            else:
                if from_angle * to_angle < 0 and from_angle_non_negative > to_angle_non_negative:
                    # only one angle is negative, so values need to cross from 358., 359., 0., 1., 2. 
                    return [range_measurement for range_measurement in self.range_measurements if from_angle_non_negative <= range_measurement.get_angle() or range_measurement.get_angle() < to_angle_non_negative]
                else:
                    # from_angle and to_angle both specified
                    return [range_measurement for range_measurement in self.range_measurements if from_angle_non_negative <= range_measurement.get_angle() < to_angle_non_negative]

    def get_all_range_measurements(self):
        return self.range_measurements

class Turtlebot3ObstacleDetection(Node):
    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        self.next_tele_twist = Twist()
        self.next_tele_twist.linear.x = 0.
        self.next_tele_twist.angular.z = 0.

        self.ranges = Ranges()
        self.ranges_have_been_measured = False

        self.MAX_LINEAR_VELOCITY    = 0.4
        self.MAX_ANGULAR_VELOCITY   = 0.67
        self.STOP_DISTANCE          = 0.15 # Nothing should get closer to the robot than this distance
        self.IGNORING_DISTANCE      = 2.0  # distance at which we don't care if something is in the way

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
        self.ranges_have_been_measured = True
        self.ranges.update_range_measurements(msg)

    def cmd_vel_raw_callback(self, msg):
        self.next_tele_twist = msg

    def timer_callback(self):
        if self.ranges_have_been_measured:
            print("Entering mainloop()")
            self.mainloop()

    def direction_of_most_open_direction(self):
        angles_of_open_direction:list[list] = []
        angles = []
        for measurement in self.ranges.get_all_range_measurements():
            if measurement.get_distance() > self.STOP_DISTANCE:
                angles.append(measurement.get_angle())
            else:
                if len(angles) > 0:
                    angles_of_open_direction.append(angles)
                angles = []
        
        angles_of_open_direction.sort(key=lambda x : len(x), reverse=True)
        
        if len(angles_of_open_direction) > 0:
            return self.average(angles_of_open_direction[0])
        else:
            return 0
        
    def average(self, list):
        sum = 0
        for element in list:
            sum += element
        if len(list) > 0:
            return sum / len(list)
        else:
            return 0

    def clockwise_or_anticlockwise(self, angle1, angle2):
        clockwise_difference        = max(angle1, angle2) - min(angle1, angle2)
        anticlockwise_difference    = max(angle1, angle2)-360 - min(angle1, angle2)

        if clockwise_difference >= anticlockwise_difference:
            return 1
        else:
            return -1

    ######### MAIN #########
    def mainloop(self):
        # if self.should_stop():
        #     print("Should stop")
        #     self.next_tele_twist.linear.x = 0.
        #     self.turn_away_from_wall()
        # else:
        #     print("Forward!")
        self.next_tele_twist.linear.x  = self.MAX_LINEAR_VELOCITY
        self.next_tele_twist.angular.z = 0.

        self.cmd_vel_pub.publish(self.next_tele_twist)

    ######### STOP LOGIC #########
    def channel_in_front_is_clear(self):
        front_slice_distance_measurements = self.ranges.get_range_measurements_slice(-89, 90) # front hemisphere
        print(front_slice_distance_measurements)

        # 1/sin(angle) * stop_distance creates a channel that's stop_distance wide. For any angle, it symbolizes the distance from the robot to the walls of that channel in a particular direction.
        for measurement in front_slice_distance_measurements:
            if not np.isclose(measurement.get_angle(), 0) and measurement.is_valid():
                distance_to_wall_in_straight_channel = 1/np.sin(measurement.get_angle())*self.STOP_DISTANCE
                print(distance_to_wall_in_straight_channel, end=" ")
                if measurement.get_distance() < min(distance_to_wall_in_straight_channel, self.IGNORING_DISTANCE):
                    print("\n")
                    return False
        print("\n")
        return True
    
    def anything_in_front_within_stop_distance(self):
        front_slice_distance_measurements = self.ranges.get_range_measurements_slice(-89, 90) # front hemisphere

        if any([measurement.get_distance() < self.STOP_DISTANCE for measurement in front_slice_distance_measurements if measurement.is_valid()]):
            return True
        else:
            return False

    def should_stop(self):
        return self.anything_in_front_within_stop_distance()
        # return self.channel_in_front_is_clear()
    
    def turn_away_from_wall(self):
        goal_angle = self.direction_of_most_open_direction()
        print("Turning to", goal_angle)
        self.next_tele_twist.angular.z = self.MAX_ANGULAR_VELOCITY * self.clockwise_or_anticlockwise(0, goal_angle)

def main(args=None):
    try:
        rclpy.init(args=args)
        turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
        rclpy.spin(turtlebot3_obstacle_detection)

    except:
        raise Exception("There was something wrong when initializing the class and spinning it.")
    
    finally:
        try:
            turtlebot3_obstacle_detection.destroy_node()
        except:
            pass

        rclpy.shutdown()

if __name__ == '__main__':
    main()