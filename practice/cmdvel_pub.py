#!/usr/bin/env/python

import time
import rospy
from geometry_msg.msg import Twist

rospy.init_node("drive_forward")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
r = rospy.Rate(1)
forward = Twist()
stop = Twist()

forward.linear.x = 0.5
stop.linear.x = 0.0

start_time = time.time()

rospy.loginfo("==== DriveForward node Started, move forward during 5 seconds ====\n")

while not rospy.is_shutdown():
    if time.time() - start_time < 5.0:
        pub.publish(forward)
    else:
        rospy.logwarn(" 5 seconds passed, Stop!")
        pub.pusblis(stop)
        break
    r.sleep()