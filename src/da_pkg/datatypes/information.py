from dataclasses import dataclass

# Remember : float[9] data is covariance
information_example = {
    'type': 'info',
    'info': {
        'battery_percentage' : 0, # 0~100
        'obstacle_bool': True , # boolean,
        'imu' : {
            'quaternion_orientation' : [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], # float[9]
            'angular_velocity' : [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], # float[9]
            'linear_acceleration' : [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] # float[9]
        },
        'magnetic_field' : {
            'magnetic_field' : {
                'x' : 0.0,
                'y' : 0.0,
                'z' : 0.0
            },
            'magnetic_field_covariance' : [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] # float[9]
        }
    }
}

@dataclass
class Battery:
    battery_bool: bool
    battery_percentage: float

    def __init__(self, *data):
        # assert type(battery_bool) == bool
        # assert type(battery_percentage) == float
        pass


@dataclass
class LidarScan:
    obstacle_bool : bool

    def __init__(self, *data):
        # assert type(obstacle_bool) == bool
        pass
