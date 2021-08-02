#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ORIGINAL CODE FROM: ttps://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_teleop/nodes/turtlebot3_teleop_key

import rospy
from geometry_msgs.msg import Twist
import sys
import select
import os
if os.name == 'nt':
  import msvcrt
else:
  import tty
  import termios
import random
import time
# from DA_agent import *

class AgentMove():
    def __init__(self, turtlebot3_model):
        self.turtlebot3_model = turtlebot3_model
        # super().__init__(turtlebot3_model)
        # self.turtlebot3_model = turtlebot3_model
        # publishing topic name: cmd_vel
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist = Twist()

        self.BURGER_MAX_LIN_VEL = 0.22
        self.BURGER_MAX_ANG_VEL = 2.84

        self.WAFFLE_MAX_LIN_VEL = 0.26
        self.WAFFLE_MAX_ANG_VEL = 1.82

        self.LIN_VEL_STEP_SIZE = 0.01
        self.ANG_VEL_STEP_SIZE= 0.1

        self.status = 0
        self.target_linear_vel = 0.0
        self.target_angular_vel= 0.0

        self.control_linear_vel= 0.0
        self.control_angular_vel = 0.0

        self.msg = """
        Control Your TurtleBot3!
        ---------------------------
        Moving around:
            w
        a    s    d
            x
        w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
        a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
        space key, s : force stop
        CTRL-C to quit\n
        """

        self.e = """
        Communications Failed\n
        """

        

    def getKey(self):
        """Recognize keyboard input"""
        if os.name == 'nt':
            if sys.version_info[0] >= 3:
                return msvcrt.getch().decode()
            else:
                return msvcrt.getch()

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            return key
        else:
            key = ''
            return key

    def vels(self):
        """Return current target linear / angular velocities"""
        return "currently:\tlinear vel %s\t angular vel %s \n" % (self.target_linear_vel, self.target_angular_vel)

    def makeSimpleProfile(self, output, input, slop):
        """
        output : control_vel
        input : target_vel
        ??? : For more subtle control?
        """
        if input > output:
            # target is bigger than current
            output = min(input, output + slop)

        elif input < output:
            # target is smaller than current
            output = max(input, output - slop)
        else:
            output = input

        return output

    def constrain(self, input, low, high):
        """Set all the input values stays in low - high boundary"""
        if input < low:
            input = low
        elif input > high:
            input = high
        else:
            input = input

        return input

    def checkLinearLimitVelocity(self, vel):
        """Set linear velocity to stay in given limit"""
        if self.turtlebot3_model == "burger":
            vel = self.constrain(
                vel, -self.BURGER_MAX_LIN_VEL, self.BURGER_MAX_LIN_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            vel = self.constrain(
                vel, -self.WAFFLE_MAX_LIN_VEL, self.WAFFLE_MAX_LIN_VEL)
        else:
            # vel = self.constrain(vel, -self.BURGER_MAX_LIN_VEL, self.BURGER_MAX_LIN_VEL) DEFAULT = WAFFLE
            vel = self.constrain(
                vel, -self.WAFFLE_MAX_LIN_VEL, self.WAFFLE_MAX_LIN_VEL)

        return vel

    def checkAngularLimitVelocity(self, vel):
        """Set angular velocity to stay in given limit"""
        if self.turtlebot3_model == "burger":
            vel = self.constrain(
                vel, -self.BURGER_MAX_ANG_VEL, self.BURGER_MAX_ANG_VEL)
        elif self.turtlebot3_model == "waffle" or self.turtlebot3_model == "waffle_pi":
            vel = self.constrain(
                vel, -self.WAFFLE_MAX_ANG_VEL, self.WAFFLE_MAX_ANG_VEL)
        else:
            #vel = self.constrain(vel, -self.BURGER_MAX_ANG_VEL, self.BURGER_MAX_ANG_VEL)
            vel = self.constrain(
                vel, -self.WAFFLE_MAX_ANG_VEL, self.WAFFLE_MAX_pyANG_VEL)

        return vel


    """
    BEWARE OF VELOCITY CONCEPTS!
    
    Think about physics, use your right-hand-rule to check the positive directions of unit vectors.
    In addition, please remind that CounterClockwise(CCW) is positive and Clockwise(CW) is negative when it comes to angular velocity.

    """

    def AddFrontVelocity(self):
        """Positive Linear Velocity Increment"""
        self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel + self.LIN_VEL_STEP_SIZE)
        self.status += 1
        print(self.vels())

    def AddBackVelocity(self):
        """Negative Linear Velocity Increment"""
        self.target_linear_vel = self.checkLinearLimitVelocity(self.target_linear_vel - self.LIN_VEL_STEP_SIZE)
        self.status += 1
        print(self.vels())
    
    def Stop(self, gradually = False):
        if gradually == False:
            """everything goes to zero"""
            self.target_linear_vel = 0.0
            self.control_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.control_angular_vel = 0.0
            print(self.vels())
        else:
            """This is just for test"""
            if self.control_linear_vel > 0:
                
                pass

    def AddCCWVelocity(self):
        """Positive Angular Velocity Increment"""
        self.target_angular_vel = self.checkAngularLimitVelocity(self.target_angular_vel + self.ANG_VEL_STEP_SIZE)
        self.status = self.status + 1
        print(self.vels())

    def AddCWVelocity(self):
        """Negative Angular Velocity Increment"""
        self.target_angular_vel = self.checkAngularLimitVelocity( self.target_angular_vel - self.ANG_VEL_STEP_SIZE)
        self.status = self.status + 1
        print(self.vels())

    def AgentPublish(self):
        self.publisher.publish(self.twist)
    
    def make_twist_data(self):
        """Formulate twist data with final control data"""
        self.twist.linear.x = self.control_linear_vel
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0

        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = self.control_angular_vel

    def terminator(self):
        """make everything go to zero"""
        self.Stop()

        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0

#######################
#For clearer mangement#
#######################
        
"""ACTUAL MANAGING AREA"""

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # Node name configuration
    rospy.init_node('turtlebot3_teleop')

    # Agent model_name configuration
    turtlebot3_model = rospy.get_param("model", "waffle_pi")

    # AgentMove instance generated
    Robot = AgentMove(turtlebot3_model)

    try:
        print(Robot.msg)
        while(1):
            # get keyboard input
            key = Robot.getKey()
            if key == 'w':
                Robot.AddFrontVelocity()
            elif key == 'x':
                Robot.AddBackVelocity()
            elif key == 'a':
                Robot.AddCCWVelocity()
            elif key == 'd':
                Robot.AddCWVelocity()
            elif key == ' ' or key == 's':
                Robot.Stop()
            else:
                if (key == '\x03'):
                    break
            
            #print msg per 20 inputs and initialize
            if Robot.status == 20:
                print(Robot.msg)
                Robot.status = 0

            # set final control linear and angular velocities
            Robot.control_linear_vel = Robot.makeSimpleProfile(Robot.control_linear_vel, Robot.target_linear_vel, (Robot.LIN_VEL_STEP_SIZE/2.0))
            Robot.control_angular_vel = Robot.makeSimpleProfile(Robot.control_angular_vel, Robot.target_angular_vel, (Robot.ANG_VEL_STEP_SIZE/2.0))
            
            # formulate twist msg
            Robot.make_twist_data()

            # publish twist via 'cmd_vel' topic
            Robot.AgentPublish()
            

    except:
        print(Robot.e)

    finally:
        Robot.terminator()
        print("Warning : Teleoperation Shutdown!\n")
        Robot.AgentPublish()

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)