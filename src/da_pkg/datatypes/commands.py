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


sound_effect_example = {
    'type': 'command',
    'sound_effect': {
        'mode': SoundEffectConstants.DEFAULT_MODE,
        'random': SoundEffectConstants.DEFAULT_RANDOM,
        'language': SoundEffectConstants.DEFAULT_LANGUAGE
    }
}


@dataclass
class SoundEffect:
    mode: int
    random: int
    language: int

    def __init__(self, *data):
        """
        mode : 0 ~ FINAL_MODE
        random : 1 ~ FINAL_RANDOM
        language : 0 ~ FINAL_LANGUAGE

        Let the current settings rely on 'random = 1 & language = 0' which are DEFAULT values.

        refer to consts/SoundEffectsConstants for further details.
        """
        if len(data) == 1 and type(data[0]) is dict:
            assert data[0]['type'] == 'sound_effect'
            assert 0 <= data[0]['sound_effect']['mode'] <= SoundEffectConstants.FINAL_MODE
            assert 1 <= data[0]['sound_effect']['random'] <= SoundEffectConstants.FINAL_RANDOM
            assert 0 <= data[0]['sound_effect']['language'] <= SoundEffectConstants.FINAL_LANGUAGE

            self.mode = int(data[0]['sound_effect']['mode'])
            self.random = int(data[0]['sound_effect']['random'])
            self.language = int(data[0]['sound_effect']['language'])

        elif len(data) == 3:
            assert 0 <= data[0] < 5
            assert 1 <= data[1] < 2
            assert 0 <= data[2] < 3
            self.mode, self.random, self.language = data
        else:
            raise TypeError

    def __iter__(self):
        yield 'type', 'command'
        yield 'sound_effect', {'mode': self.mode, 'random': self.random, 'language': self.language}


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
