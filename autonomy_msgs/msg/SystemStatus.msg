std_msgs/Header header
string          source
uint8           status
# OLD:
#uint8 OK         = 0   # The goal has yet to be processed by the action server
#uint8 HOLD       = 1   # The goal is currently being processed by the action server
#uint8 ABORT      = 2   # The goal received a cancel request after it started executing
uint8 UNDEF         = 0
uint8 OK            = 1   #
uint8 NONCRIT       = 2   #
uint8 INCONVENIENT  = 4
uint8 HOLD          = 8
uint8 ABORT         = 16

#Allow for the user to associate a string with GoalStatus for debugging
string  info
