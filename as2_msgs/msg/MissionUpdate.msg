# Message that sends a mission to the interpreter

string drone_id     # ID of the drone that will execute the mission
int32 mission_id    # ID of the mission to be executed
int32 item_id       # ID of the item to be executed

uint8 EXECUTE = 0   # Execute a mission in the interpreter
uint8 LOAD    = 1   # Load a mission to the interpreter
uint8 START   = 2   # Start the execution of a mission
uint8 PAUSE   = 3   # Pause the execution of a mission
uint8 RESUME  = 4   # Resume the execution of a mission
uint8 STOP    = 5   # Stop the execution of a mission
uint8 JUMP_TO = 6   # Jump to a specific item in the mission
uint8 REPEAT  = 7   # Repeat the execution of a mission
uint8 INSERT  = 8   # Insert an item in the mission
uint8 MODIFY  = 9   # Modify an item in the mission
uint8 REMOVE  = 10  # Remove an item in the mission
uint8 RESET   = 11  # Reset the interpreter
uint8 ABORT   = 12  # Abort the execution of a mission

uint8 action        # Action to be performed in the interpreter

string mission      # JSON formatted mission to be executed
