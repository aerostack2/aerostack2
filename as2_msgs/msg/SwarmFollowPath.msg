# Message that encodes de swarm follow path

std_msgs/Header header                  # Message header, with the frame of the pose list
as2_msgs/YawMode yaw_drones             # Yaw drones mode
as2_msgs/YawMode yaw_swarm              # Yaw swarm mode
as2_msgs/PoseWithID[] path              # List of poses with ID in path 
float32 max_speed                       # Maximum speed (m/s)     