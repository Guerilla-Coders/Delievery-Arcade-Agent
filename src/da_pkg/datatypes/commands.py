from dataclasses import dataclass

movement_example = {
    'type': 'command',
    'movement': {
        'throttle': 0,  # -32 ~ 31
        'steer': 0  # -32 ~ 31
    }
}


@dataclass
class Movement:
    throttle: int
    steer: int

    def __init__(self, *data):
        """
        throttle: 0~63
        steer: -32~31

        :param data: (throttle, steer)
        """
        if len(data) == 1 and type(data[0]) is dict:
            assert data[0]['type'] == 'command'
            assert -32 <= data[0]['movement']['throttle'] < 32
            assert -32 <= data[0]['movement']['steer'] < 32

            self.throttle = int(data[0]['movement']['throttle'])
            self.steer = int(data[0]['movement']['steer'])
        elif len(data) == 2:
            assert -32 <= data[0] < 32
            assert -32 <= data[1] < 32
            self.throttle, self.steer = data
        else:
            raise TypeError

    def __iter__(self):
        yield 'type', 'command'
        yield 'movement', {'throttle': self.throttle, 'steer': self.steer}
