# Gimbal Control message definition

uint8 POSITION_MODE = 0
uint8 SPEED_MODE = 1

uint8 control_mode
geometry_msgs/Vector3Stamped target    # x: roll y: pitch z: yaw
