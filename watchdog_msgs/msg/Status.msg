# Sensor Information
string              entity  # Entity Identifier

# Type Information
uint8               type    # Sensor Group (Topic, Node, Driver, global)
uint8 GLOBAL        = 0
uint8 TOPIC         = 1
uint8 NODE          = 2
uint8 DRIVER        = 3

# Status Information
uint8               status  # Sensor Status (any of the following)
uint8 UNDEF         = 0     # Undefined Condition
uint8 NOMINAL       = 1     # Nominal Condition
uint8 STARTING      = 2     # Startup phase of asset
uint8 DEFECT        = 4     # Inconvenient Failure
uint8 ERROR         = 8     # Severe Failure

# Debug Information
string              name    # Specific name of sensor (topic/node/driver)
string              info    # Info on the failure
