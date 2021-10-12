from dataclasses import dataclass
from ..consts import SoundEffectConstants

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
            self.throttle, self.steer = int(data[0]), int(data[1])
        else:
            raise TypeError

    def __iter__(self):
        yield 'type', 'command'
        yield 'movement', {'throttle': self.throttle, 'steer': self.steer}


sound_effects_example = {
    'type': 'command',
    'sound_effects': {
        'code': 'ALERT'
    }
}


@dataclass
class SoundEffect:
    code: str

    def __init__(self, *data):
        if len(data) == 1 and type(data[0]) is dict:
            assert data[0]['type'] == 'command'
            assert data[0]['sound_effect']['code'] in SoundEffectConstants.code
            self.code = data[0]['sound_effect']['code']

        elif len(data) == 1 and type(data[0]) is str:
            assert data[0] in SoundEffectConstants.code
            self.code = data[0]
        else:
            raise TypeError

    def __iter__(self):
        yield 'type', 'command'
        yield 'sound_effect', {'code': self.code}


lid_action_example = {
    'type': 'command',
    'lid_action': {
        'action': 'close'  # or 'open'
    }
}


@dataclass
class LidAction:
    action: str

    def __init__(self, data):
        """
        action: 'open' or 'close'
        :param data: action
        """
        if type(data) is dict:
            assert data['type'] == 'command'
            assert data['lid_action']['action'] == 'open' or data['lid_action']['action'] == 'close'

            self.action = data['lid_action']['action']
        elif type(data) is str:
            assert data == 'open' or data == 'close'
        else:
            raise TypeError

    def __iter__(self):
        yield 'type', 'command'
        yield 'lid_action', {'action': self.action}
