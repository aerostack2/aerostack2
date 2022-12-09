# Message that encodes the possible follow target info supported in Aerostack2.

std_msgs/Header header          # Message header

int8 WAITING      = 0
int8 RUNNING      = 1
int8 END          = 2

int8 follow_status              # Follow status

int8 UNSET              =  0    # Mode when the follow mode is not set
int8 PICKUP             =  1 
int8 UNPICK             =  2
int8 DYNAMIC_LAND       =  3 
int8 DYNAMIC_FOLLOWER   =  4 

int8 follow_mode                # Follow mode
