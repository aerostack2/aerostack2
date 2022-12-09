# Message that encodes the possible control modes supported in Aerostack2.

std_msgs/Header header          # Message header

int8 NONE       = 0
int8 YAW_ANGLE  = 1             # Yaw angle control mode (rad)
int8 YAW_SPEED  = 2             # Yaw speed control mode (rad/s)

int8 yaw_mode                   # Yaw mode

int8 UNSET                =  0  # mode when the controller is not set
int8 HOVER                =  1  # mode when the controller is in hover mode
int8 POSITION             =  2  # x,   y , z  refs
int8 SPEED                =  3  # vx, vy , vz refs
int8 SPEED_IN_A_PLANE     =  4  # vx, vy , z refs
int8 ATTITUDE             =  5  # roll, pitch, yaw  refs
int8 ACRO                 =  6  # roll, pitch, yawrate  refs
int8 TRAJECTORY           =  7  # x, y z, vx, vy, vz, ax, ay, az refs
int8 ACEL                 =  8  # ax, ay, az refs

int8 control_mode               # Control mode

int8 UNDEFINED_FRAME      =  0
int8 LOCAL_ENU_FRAME      =  1  # local coordinates (use this by default)
int8 BODY_FLU_FRAME       =  2  # body coordinates 
int8 GLOBAL_LAT_LONG_ASML =  3  # gnss wcoordinates 

int8 reference_frame            # Reference frame
