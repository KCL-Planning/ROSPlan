# A knowledge item used internally in ROSPlan
uint8 INSTANCE = 0
uint8 DOMAIN_ATTRIBUTE = 1
uint8 INSTANCE_ATTRIBUTE = 2
uint8 DOMAIN_FUNCTION = 3

uint8 knowledge_type

# instance knowledge_type
string instance_type
string instance_name

# attribute knowledge_type
string attribute_name
diagnostic_msgs/KeyValue[] values

# function value
float64 function_value
