time stamp
string source          # source of that status 
uint8 status
uint8 OK         = 0   # System Status is good: do operation
uint8 HOLD       = 1   # System is not ready yet: wait
uint8 ABORT      = 2   # System status is critical, issue could not be resolved! Abort


#Allow for the user to associate a string with information for debugging
string info
