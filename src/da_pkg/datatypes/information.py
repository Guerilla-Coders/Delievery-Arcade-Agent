from dataclasses import dataclass

# Remember : float[9] data is covariance
information_example = {
    'type': 'information',
    'info': {
        'battery_percentage': 0,  # 0~100
        'obstacle_detected': True,  # boolean,
        'imu': {
            'quaternion_orientation': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # float[9]
            'angular_velocity': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # float[9]
            'linear_acceleration': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # float[9]
        },
        'magnetic_field': {
            'magnetic_field': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0
            },
            'magnetic_field_covariance': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # float[9]
        },
        'laser_scan' : {
            
        }
    }
}


@dataclass
class Information:
    battery_percentage: float
    obstacle_detected: bool
    imu: dict
    magnetic_field: dict

    def __init__(self, *data):
        if len(data) == 1 and type(data[0]) is dict:
            assert data[0]['type'] == 'information'
            # TODO: additional assertion
            self.battery_percentage = data[0]['info']['battery_percentage']
            self.obstacle_detected = data[0]['info']['obstacle_detected']
            self.imu = data[0]['info']['imu']
            self.magnetic_field = data[0]['info']['magnetic_field']
        elif len(data) == 4:
            self.battery_percentage, self.obstacle_detected, self.imu, self.magnetic_field = data
        else:
            raise TypeError

    def __iter__(self):
        yield 'type', 'information'
        payload = {
            'battery_percentage': self.battery_percentage,  # 0~100
            'obstacle_detected': self.obstacle_detected,  # boolean,
            'imu': self.imu,
            'magnetic_field': self.magnetic_field
        }
        yield 'info', payload


@dataclass
class LidarScan:
    obstacle_bool: bool

    def __init__(self, *data):
        # assert type(obstacle_bool) == bool
        pass
