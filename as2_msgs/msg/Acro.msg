# Message for RPY rates and thrust (ACRO) [based on mav_msgs/msg/RateThrust.msg]

std_msgs/Header header               # Message header

geometry_msgs/Vector3 angular_rates  # Roll-, pitch-, yaw-rate around body axes [rad/s]
geometry_msgs/Vector3 thrust         # Thrust [N] expressed in the body frame.
                                     # For a fixed-wing, usually the x-component is used.
                                     # For a multi-rotor, usually the z-component is used.
                                     # Set all un-used components to 0.