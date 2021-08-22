import rospy
from std_msgs.msg import Int64
from da_pkg.msg import *
from ...consts import SoundEffectsConstants
from ...datatypes.sound_effects import SoundEffect

class SoundEffectPublisher:
    def __init__(self):
        # self.publisher = rospy.Publisher('sound', Int64, queue_size=10)
        self.publisher = rospy.Publisher('sound', sound_effects, queue_size = 10)
        # self.int64 = Int64()
        self.sound_effects = sound_effects()

        self.mode = SoundEffectsConstants.DEFAULT_MODE
        self.random = SoundEffectsConstants.DEFAULT_RANDOM
        self.language = SoundEffectsConstants.DEFAULT_LANGUAGE

    def set_soundeffect(self, soundeffect : SoundEffect):
        pass
