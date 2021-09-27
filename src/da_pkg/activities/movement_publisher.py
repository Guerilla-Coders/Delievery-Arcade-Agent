import rospy
from geometry_msgs.msg import Twist
from ..consts import Limits
from ..datatypes.commands import Movement
from ..physics_processing import constrain, make_simple_profile, map_number


class MovementPublisher:
    def __init__(self):
        # publishing topic name: cmd_vel
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Node name configuration
        rospy.init_node('Movement')
        rospy.loginfo(f"Initialized node: movement")
        self.twist = Twist()

        self.target_linear_vel = 0.0
        self.control_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_angular_vel = 0.0

    def set_movement(self, movement: Movement):
        self.target_linear_vel = map_number(movement.throttle, -32, 32,
                                            -Limits.WAFFLE_MAX_LIN_VEL, Limits.WAFFLE_MAX_LIN_VEL)
        self.target_linear_vel = constrain(self.target_linear_vel, -Limits.WAFFLE_MAX_LIN_VEL,
                                           Limits.WAFFLE_MAX_LIN_VEL)
        self.target_angular_vel = map_number(movement.steer, -32, 32,
                                             Limits.WAFFLE_MAX_ANG_VEL, -Limits.WAFFLE_MAX_ANG_VEL)
        self.target_angular_vel = constrain(self.target_angular_vel, -Limits.WAFFLE_MAX_ANG_VEL,
                                            Limits.WAFFLE_MAX_ANG_VEL)

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

    def terminate(self):
        """make everything go to zero"""
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.do_publishing()

    def run(self):
        # set final control linear and angular velocities
        self.control_linear_vel = make_simple_profile(self.control_linear_vel, self.target_linear_vel,
                                                      (Limits.LIN_VEL_STEP_SIZE / 2.0))
        self.control_angular_vel = make_simple_profile(self.control_angular_vel, self.target_angular_vel,
                                                       (Limits.ANG_VEL_STEP_SIZE / 2.0))
        # formulate twist msg
        self.make_twist_data()
        # publish twist via 'cmd_vel' topic
        self.do_publishing()
