# Message that sends a mission to the interpreter

int32 drone_id
int32 mission_id

uint8 EXECUTE = 0
uint8 STOP    = 1
uint8 APPEND  = 2
uint8 INSERT  = 3

uint8 type

string mission
