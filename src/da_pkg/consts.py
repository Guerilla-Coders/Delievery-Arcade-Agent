import rospy

rospy.loginfo("Imported consts.py")


class Limits:
    BURGER_MAX_LIN_VEL = 0.22
    BURGER_MAX_ANG_VEL = 2.84

    WAFFLE_MAX_LIN_VEL = 0.26
    WAFFLE_MAX_ANG_VEL = 1.82

    LIN_VEL_STEP_SIZE = 0.01
    ANG_VEL_STEP_SIZE = 0.1

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0

    control_linear_vel = 0.0
    control_angular_vel = 0.0
