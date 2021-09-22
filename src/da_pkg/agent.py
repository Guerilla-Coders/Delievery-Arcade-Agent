from .activities.movement_publisher import MovementPublisher
from .activities.information_subscriber import InformationSubscriber
from .activities.sound_effect_publisher import SoundEffectPublisher
from .datatypes.information import Information
import time

FREQUENT = 1
OFTEN = 5
RARELY = 20


class DeliveryArcadeAgent:
    information: Information

    def __init__(self):
        self.movement_publisher = MovementPublisher()
        self.sound_effect_publisher = SoundEffectPublisher()

        self.information_subscriber = InformationSubscriber()

        self.frequent_task_timer = 0
        self.often_task_timer = 0
        self.rarely_task_timer = 0

    def run(self) -> tuple:
        self.movement_publisher.run()
        # self.sound_effect_publisher.run()
        self.sound_effect_publisher.is_msg_going_well()

        output = [None, None, None]

        if time.time() - self.frequent_task_timer > FREQUENT:
            self.frequent_task_timer = time.time()
            self.information = self.information_subscriber.get_information()
            output[0] = self.information
        if time.time() - self.often_task_timer > OFTEN:
            self.often_task_timer = time.time()
            # output[1] = self.lidar
            pass
        if time.time() - self.rarely_task_timer > RARELY:
            self.rarely_task_timer = time.time()
            pass

        return tuple(output)

    def terminate(self):
        self.movement_publisher.terminate()
        self.sound_effect_publisher.terminate()
