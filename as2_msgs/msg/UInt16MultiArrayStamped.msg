# Please look at the std_msgs/MultiArrayLayout message definition for
# documentation on all multiarrays.
# This message is a multiarray of uint16 values with a timestamp, based on
# the std_msgs/MultiArrayLayout message.

builtin_interfaces/Time stamp     # Message timestamp
std_msgs/MultiArrayLayout layout  # Specification of data layout
uint16[] data                     # Array of data