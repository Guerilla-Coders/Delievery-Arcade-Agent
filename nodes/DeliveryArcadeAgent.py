#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import os
import select
import tty
import termios
from da_pkg.consts import Limits
from da_pkg.physics_processing import make_simple_profile, constrain
from da_pkg.network_config import NetworkConfig


def get_key():
    """키보드 좌판 입력을 받아오는 함수"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key_input = sys.stdin.read(1)
    else:
        key_input = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key_input


class DeliveryArcadeAgent:
    def __init__(self):
        # publishing topic name: cmd_vel
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Node name configuration
        rospy.init_node('turtlebot3_teleop')
        rospy.loginfo(f"Initialized node: turtlebot3_teleop")
        self.twist = Twist()

        self.target_linear_vel = 0.0
        self.control_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_angular_vel = 0.0

    def get_vels(self):
        """Return current target linear / angular velocities"""
        return f"currently:\tlinear vel {self.target_linear_vel}\t angular vel {self.target_angular_vel}"

    """
    BEWARE OF VELOCITY CONCEPTS!
    
    Think about physics, use your right-hand-rule to check the positive directions of unit vectors.
    In addition, please remind that CounterClockwise(CCW) is positive and Clockwise(CW) is negative when it comes to angular velocity.

    """

    def add_front_velocity(self):
        """Positive Linear Velocity Increment"""
        raw_target_linear_vel = self.target_linear_vel + Limits.LIN_VEL_STEP_SIZE
        self.target_linear_vel = constrain(raw_target_linear_vel, -Limits.WAFFLE_MAX_LIN_VEL, Limits.WAFFLE_MAX_LIN_VEL)
        print(self.get_vels())

    def add_back_velocity(self):
        """Negative Linear Velocity Increment"""
        raw_target_linear_vel = self.target_linear_vel - Limits.LIN_VEL_STEP_SIZE
        self.target_linear_vel = constrain(raw_target_linear_vel, -Limits.WAFFLE_MAX_LIN_VEL, Limits.WAFFLE_MAX_LIN_VEL)
        print(self.get_vels())

    def stop(self, gradually=False):
        if not gradually:
            """everything goes to zero"""
            self.target_linear_vel = 0.0
            self.control_linear_vel = 0.0
            self.target_angular_vel = 0.0
            self.control_angular_vel = 0.0
            print(self.get_vels())
        else:
            """This is just for test"""
            if self.control_linear_vel > 0:
                pass

    def add_CCW_velocity(self):
        """Positive Angular Velocity Increment"""
        raw_target_angular_vel = self.target_angular_vel + Limits.ANG_VEL_STEP_SIZE
        self.target_angular_vel = constrain(raw_target_angular_vel, -Limits.WAFFLE_MAX_ANG_VEL,
                                            Limits.WAFFLE_MAX_ANG_VEL)
        print(self.get_vels())

    def add_CW_velocity(self):
        """Negative Angular Velocity Increment"""
        raw_target_angular_vel = self.target_angular_vel - Limits.ANG_VEL_STEP_SIZE
        self.target_angular_vel = constrain(raw_target_angular_vel, -Limits.WAFFLE_MAX_ANG_VEL,
                                            Limits.WAFFLE_MAX_ANG_VEL)
        print(self.get_vels())

    def do_publishing(self):
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
        self.stop()

        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0


#######################
# For clearer mangement#
#######################

"""ACTUAL MANAGING AREA"""

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    node_dir = os.path.dirname(__file__)
    config_file_path = "network_config.json"
    abs_config_file_path = os.path.join(node_dir, config_file_path)

    print(f"Loading network config file from {abs_config_file_path}")
    network_config = NetworkConfig(abs_config_file_path)
    print(f"Control server URI: http://{network_config.ip}:{network_config.ports.control}")

    # DeliveryArcadeAgent instance generated
    Robot = DeliveryArcadeAgent()
    rospy.loginfo("Created DeliveryArcadeAgent()")

    try:
        while True:
            # get keyboard input
            key = get_key()
            if key == 'w':
                Robot.add_front_velocity()
            elif key == 'x':
                Robot.add_back_velocity()
            elif key == 'a':
                Robot.add_CCW_velocity()
            elif key == 'd':
                Robot.add_CW_velocity()
            elif key == ' ' or key == 's':
                Robot.stop()
            elif key == '\x03':
                break

            # set final control linear and angular velocities
            Robot.control_linear_vel = make_simple_profile(Robot.control_linear_vel, Robot.target_linear_vel,
                                                           (Limits.LIN_VEL_STEP_SIZE / 2.0))
            Robot.control_angular_vel = make_simple_profile(Robot.control_angular_vel, Robot.target_angular_vel,
                                                            (Limits.ANG_VEL_STEP_SIZE / 2.0))

            # formulate twist msg
            Robot.make_twist_data()

            # publish twist via 'cmd_vel' topic
            Robot.do_publishing()

    except BaseException as error:
        print(f"Exception {error}")

    finally:
        Robot.terminator()
        print("Warning : Teleoperation Shutdown!\n")
        Robot.do_publishing()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
