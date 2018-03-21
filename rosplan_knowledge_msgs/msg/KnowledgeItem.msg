# A knowledge item used to represent a piece of the state in ROSPlan

uint8 INSTANCE = 0
uint8 FACT = 1
uint8 FUNCTION = 2

uint8 knowledge_type

# time at which this knowledge becomes true
time initial_time

# instance knowledge_type
string instance_type
string instance_name

# attribute knowledge_type
string attribute_name
diagnostic_msgs/KeyValue[] values

# function value
float64 function_value

# represents a negative fact
bool is_negative
