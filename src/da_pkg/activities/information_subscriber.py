import rospy
import math
from ..datatypes.information import Information
from sensor_msgs.msg import LaserScan, BatteryState, Imu, MagneticField

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
WARNING_DISTANCE = 0.5

MAX_THRESHOLD = 3.5
MIN_THRESHOLD = 0


class InformationSubscriber:
    def __init__(self):
        self.battery_voltage = 0.0

        self.lidar_distances = []
        self.min_distance = 0.0
        self.obstacle_state = False

        self.quaternion_orientation = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.linear_acceleration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.magnetic_field_x = 0.0
        self.magnetic_field_y = 0.0
        self.magnetic_field_z = 0.0
        self.magnetic_field_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def fetch_battery_data(self) -> None:
        battery_data: BatteryState = rospy.wait_for_message('battery_state', BatteryState)
        self.battery_voltage = battery_data.voltage

    def fetch_scan_data(self) -> None:
        """return a list that contains information of laser-scanned samples with index of each angles."""
        scan: LaserScan = rospy.wait_for_message('scan', LaserScan)
        scan_filter = []

        samples = len(scan.ranges)
        samples_view = 1  # 1 <= samples_view <= samples

        if samples_view > samples:  # samples_views will be classified as obstacles
            samples_view = samples

        if samples_view == 1:  # only 1 sample_views
            scan_filter.append(scan.ranges[0])

        else:  # if sample view is lower than samples -> reset sample ranges and get info from the sensor
            left_lidar_samples_ranges = -(samples_view // 2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view // 2

            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = MAX_THRESHOLD
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = MIN_THRESHOLD

        self.lidar_distances = scan_filter
        self.min_distance = min(scan_filter)
        self.obstacle_state = self.min_distance < WARNING_DISTANCE

    def fetch_imu_data(self) -> None:
        imu_data: Imu = rospy.wait_for_message('imu', Imu)
        self.quaternion_orientation = imu_data.orientation
        self.angular_velocity = imu_data.angular_velocity
        self.linear_acceleration = imu_data.linear_acceleration

    def fetch_magnetic_data(self) -> None:
        magnetic_data: MagneticField = rospy.wait_for_message('magnetic_field', MagneticField)
        self.magnetic_field_x = magnetic_data.magnetic_field.x
        self.magnetic_field_y = magnetic_data.magnetic_field.y
        self.magnetic_field_z = magnetic_data.magnetic_field.z
        self.magnetic_field_covariance = magnetic_data.magnetic_field_covariance

    def get_imu_dict(self) -> dict:
        return {
            'quaternion_orientation': list(self.quaternion_orientation),  # float[9]
            'angular_velocity': list(self.angular_velocity),  # float[9]
            'linear_acceleration': list(self.linear_acceleration)  # float[9]
        }

    def get_magnetic_dict(self) -> dict:
        return {
            'magnetic_field': {
                'x': self.magnetic_field_x,
                'y': self.magnetic_field_y,
                'z': self.magnetic_field_z
            },
            'magnetic_field_covariance': self.magnetic_field_covariance
        }

    def get_information(self) -> Information:
        self.fetch_battery_data()
        self.fetch_scan_data()
        information = Information(
            self.battery_voltage, self.obstacle_state, self.get_magnetic_dict(), self.get_magnetic_dict()
        )
        return information
