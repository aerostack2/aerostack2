# Message that sends a mission to the interpreter

int32 drone_id
int32 mission_id
int32 item_id

uint8 EXECUTE = 0
uint8 LOAD    = 1
uint8 START   = 2
uint8 PAUSE   = 3
uint8 RESUME  = 4
uint8 STOP    = 5
uint8 JUMP_TO = 6
uint8 REPEAT  = 7
uint8 INSERT  = 8
uint8 MODIFY  = 9
uint8 REMOVE  = 10
uint8 RESET   = 11
uint8 ABORT   = 12

uint8 type

string mission
