from dataclasses import dataclass
from ..consts import SoundEffectsConstants

sound_effects_example = {
    'type' : 'sound_effects',
    'sound_effects' : {
        'mode' : SoundEffectsConstants.DEFAULT_MODE,
        'random' : SoundEffectsConstants.DEFAULT_RANDOM,
        'language' : SoundEffectsConstants.DEFAULT_LANGUAGE
    }
}


@dataclass
class SoundEffect:
    mode : int
    random : int
    language : int

    def __init__(self, *data):
        """
        mode : 0 ~ FINAL_MODE
        random : 1 ~ FINAL_RANDOM
        language : 0 ~ FINAL_LANGUAGE

        Let the current settings rely on 'random = 1 & language = 0' which are DEFAULT values.
        
        refer to consts/SoundEffectsConstants for further details.
        """
        if len(data) == 1 and type(data[0]) is dict:
                assert data[0]['type'] == 'sound_effects'
                assert 0 <= data[0]['sound_effects']['mode'] <= SoundEffectsConstants.FINAL_MODE
                assert 1 <= data[0]['sound_effects']['random'] <= SoundEffectsConstants.FINAL_RANDOM
                assert 0 <= data[0]['sound_effects']['language'] <= SoundEffectsConstants.FINAL_LANGUAGE


                self.mode = int(data[0]['sound_effects']['mode'])
                self.random = int(data[0]['sound_effects']['random'])
                self.language = int(data[0]['sound_effects']['language'])

        # ?????????????????
        elif len(data) == 2:
            assert -32 <= data[0] < 32
            assert -32 <= data[1] < 32
            self.mode, self.random, self.language = data
        else:
            raise TypeError

    def __iter__(self):
        yield 'type', 'sound_effects'
        yield 'sound_effects', {'mode': self.mode, 'random': self.random, 'language' : self.language}