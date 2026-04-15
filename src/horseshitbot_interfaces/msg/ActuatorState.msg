uint8 STATE_IDLE=0
uint8 STATE_REFERENCING=1
uint8 STATE_REFERENCED=2
uint8 STATE_MOVING_OPEN=3
uint8 STATE_MOVING_CLOSE=4
uint8 STATE_ERROR=5

uint8 state
bool is_referenced
string direction
float32 speed_rpm
string error_message
