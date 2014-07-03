# A knowledge item used internally in ROSPlan
uint8 INSTANCE = 0
uint8 ATTRIBUTE = 1

uint8 knowledge_type

# instance knowledge_type
string instance_type
string instance_name

# instance attribute knowledge_type
string attribute_name
diagnostic_msgs/KeyValue[] values
