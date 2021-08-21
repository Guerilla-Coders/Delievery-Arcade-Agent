import rospy
import math
from sensor_msgs.msg import LaserScan, BatteryState, Imu, MagneticField

LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
WARNING_DISTANCE = 0.5

MAX_THRESHOLD = 3.5
MIN_THRESHOLD = 0

class MagneticSubscriber():
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

    def fetch_magnetic_data(self) -> None:
        magnetic_data: MagneticField = rospy.wait_for_message('magnetic_field', MagneticField)
        self.magnetic_field_x = magnetic_data.magnetic_field.x
        self.magnetic_field_y = magnetic_data.magnetic_field.y
        self.magnetic_field_z = magnetic_data.magnetic_field.z
        self.magnetic_field_covariance = magnetic_data.magnetic_field_covariance

    def get_magnetic_dict(self) -> dict:
        return {
            'magnetic_field': {
                'x': self.magnetic_field_x,
                'y': self.magnetic_field_y,
                'z': self.magnetic_field_z
            },
            'magnetic_field_covariance': self.magnetic_field_covariance
        }

    # def get_information(self) -> MagneticInformation:
    #     magnetic_information = MagneticInformation(
    #         self.get_magnetic_dict()
    #     )
    #     return magnetic_information
