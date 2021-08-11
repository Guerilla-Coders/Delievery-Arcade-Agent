import rospy
from geometry_msgs.msg import Twist
from .consts import Limits
from .physics_processing import constrain, make_simple_profile


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

        self.last_target_linear_vel = 0.0
        self.last_control_linear_vel = 0.0
        self.last_target_angular_vel = 0.0
        self.last_control_angular_vel = 0.0

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

    def run(self):
        if self.target_linear_vel != self.last_target_linear_vel or \
                self.target_angular_vel != self.last_target_angular_vel or \
                self.control_linear_vel != self.last_control_linear_vel or \
                self.control_angular_vel != self.last_control_angular_vel:
            rospy.logdebug(f"TARGET {self.target_linear_vel} {self.target_angular_vel}")
            rospy.logdebug(f"CONTROL {self.control_linear_vel} {self.control_angular_vel}")
            self.last_target_linear_vel = self.target_linear_vel
            self.last_target_angular_vel = self.target_angular_vel
            self.last_control_linear_vel = self.control_linear_vel
            self.last_control_angular_vel = self.control_angular_vel
        # set final control linear and angular velocities
        self.control_linear_vel = make_simple_profile(self.control_linear_vel, self.target_linear_vel,
                                                      (Limits.LIN_VEL_STEP_SIZE / 2.0))
        self.control_angular_vel = make_simple_profile(self.control_angular_vel, self.target_angular_vel,
                                                       (Limits.ANG_VEL_STEP_SIZE / 2.0))

        # formulate twist msg
        self.make_twist_data()

        # publish twist via 'cmd_vel' topic
        self.do_publishing()
