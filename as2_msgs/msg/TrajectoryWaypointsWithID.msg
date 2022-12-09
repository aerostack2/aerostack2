# Message for generating a trajectory for a given path of poses with ID  
# with a given yaw mode and velocity

std_msgs/Header header                  # Message header

uint8 KEEP_YAW             = 0
uint8 PATH_FACING          = 1
uint8 GENERATE_YAW_TRAJ    = 2
uint8 YAW_FROM_TOPIC       = 3

uint8 yaw_mode                          # Yaw mode for trajectory

as2_msgs/PoseStampedWithID[] poses      # List of poses with ID in trajectory
