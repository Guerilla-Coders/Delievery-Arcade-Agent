#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import BatteryState
from turtlebot3_msgs.msg import SensorState
from std_msgs.msg import Bool

class DeliveryArcadeAgent_State():
    def __init__(self):
        rospy.init_node('DeliveryArcadeAgent_State')
        rospy.loginfo("Initialized node: DelieveryArcadeAgent_State")
        
        self.battery = DeliveryArcadeAgent_Battery()
        self.obstacle = DeliveryArcadeAgent_Obstacle()



class DeliveryArcadeAgent_Battery:
    def __init__(self):
        """
        FOR TEST : subscriber should not be done like this!
        """
        # self.battery_subscriber = self.battery_state_listener() 
        self.battery_state = True
        self.battery_percentage = 0.0

     def battery_callback(self,data):
        # print(data)
        # print('\n')
        # print(data.battery) #12.59000 -> BatteryState.voltage
        self.battery_percentage = data.percentage
        if self.battery_percentage < 0.05:
            self.battery_state = False
            rospy.loginfo(f"Warning : Battery is under {self.battery_percentage*100}%\n Please charge your battery immediately!")
        
        elif self.battery_percentage <= 0.15:
            self.battery_state = False
            rospy.loginfo(f"Warning : Battery is under {self.battery_percentage*100}%")

    def battery_state_listener(self):   
        rospy.Subscriber('battery_state', BatteryState, self.battery_callback)
        # rospy.Subscriber('topic_name', msg_type, callback)
        rospy.spin()

    def is_battery_okay(self) -> bool:
        return self.battery_state

    def run(self):
        self.battery_state_listener()
        return self.is_battery_okay()



class DeliveryArcadeAgent_Obstacle:
    def __init__(self):
        self.obstacle_state = True

    def obstacle_state_listener(self):
        rospy.Subscriber('obstacle_boolean', Bool, self.obstacle_callback)

    def obstacle_callback(self, obstacle):
        if obstacle:
            rospy.loginfo("Warning : Potential obstacle is near!")
            self.obstacle_state = False        
        else:
            self.obstacle_state = True

    def is_obstacle_okay(self)->bool:
        return self.obstacle_state

    def run(self):
        self.obstacle_state_listener
        return self.is_obstacle_okay

if __name__ == "__main__":
    RobotState = DeliveryArcadeAgent_State()
    RobotState.obstacle.run()