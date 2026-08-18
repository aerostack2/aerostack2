# Definition of a point of a trajectory

string id                  # Unique identifier for the trajectory point

geometry_msgs/Vector3 position      # Position of the vehicle in the frame_id frame
geometry_msgs/Vector3 twist         # Twist of the vehicle in the frame_id frame
geometry_msgs/Vector3 acceleration  # Acceleration of the vehicle in the frame_id frame

float32 yaw_angle                   # Yaw angle of the vehicle (rad) in the frame_id frame
