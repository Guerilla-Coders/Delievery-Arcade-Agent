import rospy
from std_msgs.msg import String
from ..datatypes.commands import LidAction
from ..consts import ActionConstants as AC


class ActionPublisher:
    def __init__(self):
        self.lid_action_publisher = rospy.Publisher('lid_action', String, queue_size=10)

        self.lid_action = None

    def set_action(self, command):
        if type(command) is LidAction:
            if command.action == "open":
                self.lid_action = String(AC.lid_open)
            elif command.action == "close":
                self.lid_action = String(AC.lid_close)
            else:
                raise TypeError
            print(f"Set Lid Action to {self.lid_action}")
        else:
            raise TypeError

    def do_publishing(self):
        if self.lid_action is not None:
            self.lid_action_publisher.publish(self.lid_action)
            print(f"Published Lid Action: {self.lid_action}")
            self.lid_action = None

    def terminate(self):
        self.lid_action = String(AC.lid_close)
        self.do_publishing()

    def run(self):
        self.do_publishing()
