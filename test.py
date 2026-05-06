class RangeMeasurement:
    MINIMUM_MEASURABLE_DISTANCE = 0.  # values below this are definitely faulty
    MAXIMUM_MEASURABLE_DISTANCE = 3.5 # values above this are assumed to be faulty

    def __str__(self):
        return f"Angle: {self.angle:<10} Distance: {self.distance:<5}"

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

    def is_valid(self) -> bool:
        """If a distance was negative, it's flagged as invalid
        """
        return self.valid

    def get_distance(self) -> float:
        """Returns a range measurement's distance value.
        """
        return self.distance

    def get_unclamped_distance(self) -> float:
        """Returns a range measurement's distance value from before it was clamped.
        """
        return self.measured_distance

class Ranges:
    def __init__(self):
        self.range_measurements = [
            RangeMeasurement(355.5,1),
            RangeMeasurement(356,2),
            RangeMeasurement(357,3),
            RangeMeasurement(358,4),
            RangeMeasurement(359,5),
            RangeMeasurement(0,6),
            RangeMeasurement(1,7),
            RangeMeasurement(2,8),
            RangeMeasurement(3,9),
            RangeMeasurement(4,10),
            RangeMeasurement(5,11)
        ]

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
                return [range_measurement for range_measurement in self.range_measurements if range_measurement.angle < to_angle_non_negative]
        else:
            # from_angle specified
            if to_angle == None:
                # only from_angle specified
                return [range_measurement for range_measurement in self.range_measurements if from_angle_non_negative <= range_measurement.angle]
            else:
                if from_angle * to_angle < 0 and from_angle_non_negative > to_angle_non_negative:
                    # only one angle is negative, so values need to cross from 358., 359., 0., 1., 2. 
                    return [range_measurement for range_measurement in self.range_measurements if from_angle_non_negative <= range_measurement.angle or range_measurement.angle < to_angle_non_negative]
                else:
                    # from_angle and to_angle both specified
                    return [range_measurement for range_measurement in self.range_measurements if from_angle_non_negative <= range_measurement.angle < to_angle_non_negative]

ranges = Ranges()
for range in ranges.get_range_measurements_slice(-2, 10):
    print(range)