import rospy
from std_msgs.msg import String
from da_pkg.msg import speech
from ..consts import SoundEffectConstants as SEC
from ..consts import SpeechData
from ..datatypes.commands import SoundEffect
from random import randrange


class SoundEffectPublisher:
    """Publish sound_effect msg from server to agent(raspberrypi)"""
    language: str
    speech_line: str
    audio_code: str

    def __init__(self):
        self.speech_publisher = rospy.Publisher('speech', speech, queue_size=10)
        self.audio_publisher = rospy.Publisher('audio', String, queue_size=10)

        self.speech = None
        self.audio = None

        self.language = SEC.english
        self.speech_line = SEC.silence
        self.audio_code = SEC.silence

    def set_soundeffect(self, soundeffect: SoundEffect):
        """Set soundeffect data obtained from app"""
        self.language = SEC.english
        if soundeffect.code in SEC.speech_needed_code:
            candidates = SpeechData.lines[self.language][soundeffect.code]
            self.speech_line = candidates[randrange(len(candidates))]
            print(f"Set Speech to {self.speech_line}")
        elif soundeffect.code in SEC.audio_needed_code:
            self.audio_code = soundeffect.code
            print(f"Set Audio Code to {self.audio_code}")
        else:
            raise AssertionError

    def do_publishing(self):
        """Publish"""
        if not (self.speech is None):
            self.speech_publisher.publish(self.speech)
            print(f"Published speech {self.speech}")
            self.speech = None
        if not (self.audio is None):
            self.audio_publisher.publish(self.audio)
            print(f"Published audio code {self.audio}")
            self.audio = None

    def terminate(self):
        """Set everything to default"""
        self.language = SEC.english
        self.speech_line = SEC.silence
        self.audio_code = SEC.silence
        self.do_publishing()

    def make_sound_effect_data(self):
        """Formulate sound_effect data"""
        if self.speech_line != SEC.silence:
            self.speech = speech(self.language, self.speech_line)
            print(f"Made speech data {self.speech}")
            self.speech_line = SEC.silence
        if self.audio_code != SEC.silence:
            self.audio = String(self.audio_code)
            print(f"Made audio code {self.audio}")
            self.audio_code = SEC.silence

    def run(self):
        self.make_sound_effect_data()
        self.do_publishing()
