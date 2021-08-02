#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int64, Bool, String

BATTERY = 100

def battery_info(battery):
    pub = rospy.Publisher('chatter', Int64, queue_size = 10)
    rospy.init_node('battery_info', anonymous = True)
    rate = rospy.Rate(100) #publish for 100 Hz rate
    while not rospy.is_shutdown():
        battery_int = BATTERY
        rospy.loginfo(battery_int)
        pub.publish(battery_int)
        rate.sleep()


def low_battery_alert(battery):
    pub = rospy.Publisher('chatter', Bool, queue_size = 10)
    rospy.init_node('low_battery_alert', anonymous = True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        if battery <= 15:
            battery_alert = True
        else:
            battery_alert = False
        rospy.loginfo(battery_alert)
        pub.publish(battery_alert)
        rate.sleep()

def battery_information_msg(battery, alert):
    pub = rospy.Publisher('chatter', String, queue_size = 10)
    rospy.init_node('battery_information_msg', anonymous = True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        battery_information_string = 'Current battery status is %d'%battery
        if alert:
            battery_information_string += ' LOW BATTERY!'
        rospy.loginfo(battery_information_string)
        pub.publish(battery_information_string)
        rate.sleep()

if __name__ == "__main__":
    try:
        battery_info(BATTERY)
        # bool = low_battery_alert(BATTERY)
        # battery_information_msg(BATTERY, bool)
    except rospy.ROSInterruptException:
        pass
