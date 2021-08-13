#!/usr/bin/env python
import rospy

from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from turtlebot3_msgs.msg import *

TOPIC = '/scan'
MSG_TYPE = LaserScan


def callback(data):
    rospy.loginfo(data)


def listener():
    rospy.init_node('topic_checker', anonymous = True)
    rospy.Subscriber(TOPIC, MSG_TYPE, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()