class Limits:
    WAFFLE_MAX_LIN_VEL = 0.26
    WAFFLE_MAX_ANG_VEL = 1.82

    LIN_VEL_STEP_SIZE = 0.01
    ANG_VEL_STEP_SIZE = 0.1

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0

    control_linear_vel = 0.0
    control_angular_vel = 0.0

class SoundEffectsConstants:
    """mode"""
    GREETING = 0
    APOLOGY = 1
    APPRECIATION = 2
    YIELD = 3
    PIGEON = 4

    DEFAULT_MODE = 0
    FINAL_MODE = 4

    """stochasiticity"""
    RANDOM = 1

    DEFAULT_RANDOM = 1
    FINAL_RANDOM = 1

    """language"""
    ENGLISH = 0
    KOREAN = 1
    SPANISH = 2

    DEFAULT_LANGUAGE = 0
    FINAL_LANGUAGE = 2