# Outcome of the flight governor (as2_flight_governor) for one motion reference.
# One message is published for every reference the governor forwards.

uint8 ALLOW = 0                     # Reference forwarded unchanged
uint8 CLAMP = 1                     # Reference modified to comply with the policies
uint8 VETO  = 2                     # Reference replaced by a hold

std_msgs/Header header              # Stamp of the reference; frame_id is the policy frame

uint8 verdict                       # ALLOW, CLAMP or VETO
bool latched                        # True while a latched veto is active
bool governed                       # False if the reference type or control mode is not governed
bool sent                           # False if nothing was forwarded (no safe hold existed yet)

string command_type                 # "pose", "twist", "speed_limit", "trajectory" or "thrust"
string[] policies                   # Names of the policies that clamped or vetoed
string reason                       # Human-readable explanation

# Values in the policy frame. For trajectories, the first setpoint.
# For "speed_limit", the velocity fields hold per-axis limits (inf = unlimited).
geometry_msgs/Vector3 requested_position
geometry_msgs/Vector3 requested_velocity
geometry_msgs/Vector3 allowed_position
geometry_msgs/Vector3 allowed_velocity
