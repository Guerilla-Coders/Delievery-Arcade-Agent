from dataclasses import dataclass

information_example = {
    'type': 'info',
    'info': {
        'battery_bool': True,  # boolean
        'battery_percentage' : 15, # 0~100
        'obstacle_bool': True  # boolean
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
