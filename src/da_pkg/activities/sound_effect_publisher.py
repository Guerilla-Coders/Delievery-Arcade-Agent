import rospy
from da_pkg.msg import sound_effect
from ..consts import SoundEffectConstants
from ..datatypes.commands import SoundEffect


class SoundEffectPublisher:
    """Publish sound_effect msg from server to agent(raspberrypi)"""

    def __init__(self):
        self.publisher = rospy.Publisher('sound_effect', sound_effect, queue_size=10)
        self.sound_effect = sound_effect()

        self.mode = SoundEffectConstants.DEFAULT_MODE
        self.random = SoundEffectConstants.DEFAULT_RANDOM
        self.language = SoundEffectConstants.DEFAULT_LANGUAGE

    def set_soundeffect(self, soundeffect: SoundEffect):
        """Set soundeffect data obtained from app"""
        self.mode = soundeffect.mode
        self.random = soundeffect.random
        self.language = soundeffect.language

    def do_publishing(self):
        """Publish"""
        self.publisher.publish(self.sound_effect)

    def terminate(self):
        """Set everything to default"""
        self.mode = SoundEffectConstants.DEFAULT_MODE
        self.random = SoundEffectConstants.DEFAULT_RANDOM
        self.language = SoundEffectConstants.DEFAULT_LANGUAGE
        self.do_publishing()

    def make_sound_effect_data(self):
        """Formulate sound_effect data"""
        self.sound_effect.mode = self.mode
        self.sound_effect.random = self.random
        self.sound_effect.language = self.language

    """Just in case!"""

    # def make_sound_effect_data(self, soundeffect : SoundEffect):
    #     """Formulate sound_effect data"""
    #     self.sound_effect.mode = soundeffect.mode
    #     self.sound_effect.random = soundeffect.random
    #     self.sound_effect.language = soundeffect.language

    def is_msg_going_well(self):
        self.sound_effect.mode = SoundEffectConstants.DEFAULT_MODE
        self.sound_effect.language = SoundEffectConstants.DEFAULT_LANGUAGE
        self.sound_effect.random = SoundEffectConstants.DEFAULT_RANDOM
        self.do_publishing()
        # rospy.loginfo(f'sound_effect msg is going well! \n{self.sound_effect}')

    def run(self):
        self.make_sound_effect_data()
        self.do_publishing()
