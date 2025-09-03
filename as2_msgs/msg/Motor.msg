# Message for RPY rates and thrust (ACRO) [based on mav_msgs/msg/RateThrust.msg]

std_msgs/Header header               # Message header

float64[] motor_speeds  #  angular speed for each motor [rad/s]
actuator_msgs/ActuatorsAngularVelocity angular_velocities  # using builtin actuators_msgs [rad/s]
