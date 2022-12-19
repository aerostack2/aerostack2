from enum import Enum


class ExtendedEnum(Enum):

    @classmethod
    def list(cls):
        return list(map(lambda c: c.value, cls))


class KeyMappings(ExtendedEnum):
    TAKE_OFF_KEY = "t"
    LAND_KEY = "l"
    HOVER_KEY = "space"
    EMERGENCY_KEY = "Delete"
    UP_KEY = "w"
    DOWN_KEY = "s"
    ROTATE_RIGHT_KEY = "d"
    ROTATE_LEFT_KEY = "a"
    LEFT_KEY = "Left"
    RIGHT_KEY = "Right"
    FORWARD_KEY = "Up"
    BACKWARD_KEY = "Down"


class ControlValues(ExtendedEnum):
    SPEED_VALUE = 1.00
    VERTICAL_VALUE = 1.00
    TURN_SPEED_VALUE = 0.10
    POSITION_VALUE = 1.00
    ALTITUDE_VALUE = 1.00
    TURN_ANGLE_VALUE = 0.10
    PITCH_ANGLE_VALUE = 0.20
    ROLL_ANGLE_VALUE = 0.20
    ATTITUDE_DURATION = 0.50


class ControlModes(ExtendedEnum):
    SPEED_CONTROL = "-SPEED-"
    POSE_CONTROL = "-POSE-"
    ATTITUDE_CONTROL = "-ATTITUDE-"


class AvailableBehaviors(ExtendedEnum):
    BEHAVIOR_TAKE_OFF = "Behavior Take Off"
    BEHAVIOR_LAND = "Behavior Land"
    BEHAVIOR_FOLLOW_PATH = "Behavior Follow Path"
    BEHAVIOR_GO_TO = "Behavior Go To"
