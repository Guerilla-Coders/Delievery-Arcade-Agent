#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(dt):
    print('-------------------------------------------')
    print('Range data at 0 deg:   {}'.format(dt.ranges[0]))
    print('Range data at 15 deg:  {}'.format(dt.ranges[15]))
    print('Range data at 345 deg: {}'.format(dt.ranges[345]))
    print('-------------------------------------------')
    
    thr1 = 0.8
    thr2 = 0.8

    if dt.ranges[0]>thr1 and dt.ranges[15]>thr2 and dt.ranges[345]>thr2: 
        pass
    if dt.ranges[0]<0.1 and dt.ranges[15]<0.1 and dt.ranges[345]<0.1:
        print("Warning: Emergency Stop!")
        move.linear.x = 0
        move.linear.y = 0
        move.angular.z = 0
        pub.publish(move) 
    else:
        print("Warning: Collision Alert!")

move = Twist()
rospy.init_node('obstacle_warning_node')
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

sub = rospy.Subscriber("/scan", LaserScan, callback)
rospy.spin()