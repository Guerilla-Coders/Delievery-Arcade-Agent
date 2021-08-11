#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import BatteryState
from turtlebot3_msgs.msg import SensorState

def callback(data):
    print(data)
    print('\n')
    # print(data.battery) #12.59000 -> BatteryState.voltage
    # rospy.loginfo(rospy.get_caller_id() + 'battery_state received: %d', data.data)

def battery_state_listener():
    rospy.init_node('battery_listener', anonymous = True)    
    rospy.Subscriber('battery_state', BatteryState, callback)
    # rospy.Subscriber('topic_name', msg_type, callback)
    # EX) rospy.Subscriber('chatter', String, callback_String)

    # rospy.init_node('turtlebot3_sensor_state_listener', anonymous = True)
    # rospy.Subscriber('sensor_state', SensorState, callback)
    rospy.spin()


class Agent():
    def __init__(self, turtlebot3_model):
        self.turtlebot3_model = turtlebot3_model
        self.battery_state = 0
        self.sensor_state = 0

    def get_battery_state(self):
        self.battery_state = callback(data)
        return self.battery_state

if __name__ == '__main__':
    battery_state_listener()
    # callback()
