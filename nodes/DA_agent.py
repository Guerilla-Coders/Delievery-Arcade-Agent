#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import BatteryState

def callback(data):
    print(type(data))
    print(type(data.data))
    # rospy.loginfo(rospy.get_caller_id() + 'battery_state received: %d', data.data)

def battery_state_listener():
    rospy.init_node('battery_listener', anonymous = True)    
    # rospy.Subscriber('topic_name', msg_type, callback)
    # EX) rospy.Subscriber('chatter', String, callback_String)
    rospy.Subscriber('battery_state', BatteryState, callback)
    rospy.spin()


class Agent():
    def __init__(self, turtlebot3_model):
        self.turtlebot3_model = turtlebot3_model
        self.battery_state = 0
        self.sensor_state = 0

    def get_battery_state(self):
        # self.battery_state = callback(data)
        # return self.battery_state

# if __name__ == '__main__':
#     battery_state_listener()
#     callback()
