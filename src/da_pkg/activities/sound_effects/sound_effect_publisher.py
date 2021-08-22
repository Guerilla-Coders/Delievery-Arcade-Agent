import rospy
# from std_msgs.msg import Int64
from da_pkg.msg import *
from ...consts import SoundEffectsConstants
from ...datatypes.sound_effects import SoundEffect

class SoundEffectPublisher:
    """Publish sound_effects msg from server to agent(raspberrypi)"""
    def __init__(self):
        """Ver 1.0 use std_msgs"""
        # self.publisher = rospy.Publisher('sound_effects', Int64, queue_size=10)
        # self.int64 = Int64()

        """Ver 2.0 : use own msg type"""
        self.publisher = rospy.Publisher('sound_effects', sound_effects, queue_size = 10)
        self.sound_effects = sound_effects()

        self.mode = SoundEffectsConstants.DEFAULT_MODE
        self.random = SoundEffectsConstants.DEFAULT_RANDOM
        self.language = SoundEffectsConstants.DEFAULT_LANGUAGE

    def set_soundeffect(self, soundeffect : SoundEffect):
        """Set soundeffect data obtained from app"""
        self.mode = soundeffect.mode
        self.random = soundeffect.random
        self.language = soundeffect.language
        
    def do_publishing(self):
        """Publish"""
        self.publisher.publish(self.sound_effects)

    def terminate(self):
        """Set everything to default"""
        self.mode = SoundEffectsConstants.DEFAULT_MODE
        self.random = SoundEffectsConstants.DEFAULT_RANDOM
        self.language = SoundEffectsConstants.DEFAULT_LANGUAGE
        self.do_publishing()
    
    def make_sound_effects_data(self):
        """Formulate sound_effects data"""
        self.sound_effects.mode = self.mode
        self.sound_effects.random = self.random
        self.sound_effects.language = self.language


    """Just in case!"""
    # def make_sound_effects_data(self, soundeffect : SoundEffect):
    #     """Formulate sound_effects data"""
    #     self.sound_effects.mode = soundeffect.mode
    #     self.sound_effects.random = soundeffect.random
    #     self.sound_effects.language = soundeffect.language

    def run(self):
        self.make_sound_effects_data()
        self.do_publishing()
