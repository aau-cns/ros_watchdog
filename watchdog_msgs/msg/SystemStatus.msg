# Sensor Information
uint8               id      # Sensor ID
string              name    # Sensor Name

# Status Information
uint8               status  # Sensor Status (any of the following)
uint8 UNDEF         = 0     # Undefined Condition
uint8 NOMINAL       = 1     # Nominal Condition
uint8 STARTING      = 2     # Startup phase of asset
uint8 DEFECT        = 4     # Inconvenient Failure
uint8 ERROR         = 8     # Severe Failure

# Debug Information
string  info
