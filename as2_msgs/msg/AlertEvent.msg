# Message that encodes different alert Events that can be handled by AS2 framework

# if value is < 0 then this alert will be handled by the platform directly
int8 KILL_SWITCH = -1
int8 EMERGENCY_HOVER = -2
int8 EMERGENCY_LAND = -3

# 0 value is used for ping or info alerts 
int8 INFO_ALERT = 0

#if value is > 0 then this alert will be handled by the AS2 framework
int8 FORCE_HOVER = 1
int8 FORCE_LAND = 2

int8 alert
string description # Further description of the alert, for debugging purposes mainly

