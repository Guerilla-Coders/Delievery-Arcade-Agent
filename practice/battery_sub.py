#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64, Bool, String

def callback_String(data):
    rospy.loginfo(rospy.get_caller_id() + 'Data received: %s', data.data)

def callback_Int64(data):
    rospy.loginfo(rospy.get_caller_id() + 'Data received: %s', data.data)

def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber('chatter', Int64, callback_Int64)
    # rospy.Subscriber('chatter', String, callback_String)
    rospy.spin()

if __name__ == '__main__':
    listener()