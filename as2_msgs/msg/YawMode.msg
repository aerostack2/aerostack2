# Yaw goal

uint8 KEEP_YAW             = 0 # Keep the current yaw angle
uint8 PATH_FACING          = 1 # Yaw angle is aligned with the path
uint8 FIXED_YAW            = 2 # Yaw angle is fixed to a given angle
uint8 YAW_FROM_TOPIC       = 3 # Yaw angle is set by a topic

uint8 mode      # Yaw mode
float32 angle   # Fixed yaw (rad)