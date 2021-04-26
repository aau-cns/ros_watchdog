# Sensor Information
uint8               id      # Sensor ID
string              name    # Sensor Name

# Status Information
uint8               status  # Sensor Status (any of the following)
uint8 UNDEF         = 0     # Undefined Condition
uint8 NOMINAL       = 1     # Nominal Condition
uint8 NONCRITICAL   = 2     # Non-Critical Failure
uint8 INCONVENIENT  = 4     # Inconvenient Failure
uint8 HOLD          = 8     # Severe Failure/Mission Hold
uint8 ABORT         = 16    # Error/Mission Abort

# Debug Information
string  info
