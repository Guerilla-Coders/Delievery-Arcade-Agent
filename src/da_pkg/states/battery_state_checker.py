#!/usr/bin/env python
import rospy
from sensor_msgs.msg import BatteryState, Imu


def callback(data):
    rospy.loginfo(data)


def listener():
    rospy.init_node('imu_checker', anonymous=True)
    rospy.Subscriber('imu', Imu, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
