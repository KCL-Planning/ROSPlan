# A knowledge item used to represent a piece of the state in ROSPlan

uint8 INSTANCE = 0
uint8 FACT = 1
uint8 FUNCTION = 2
uint8 EXPRESSION = 3
uint8 INEQUALITY = 4

uint8 knowledge_type

# time at which this knowledge becomes true
time initial_time

# knowledge is explicitly false
bool is_negative

#---------
# INSTANCE
#---------

# instance knowledge_type
string instance_type
string instance_name

#----------------------
# PREDICATE OR FUNCTION
#----------------------

# attribute knowledge_type
string attribute_name
diagnostic_msgs/KeyValue[] values

#---------
# FUNCTION
#---------

# function value
float64 function_value

# assignment operator
uint8 AP_ASSIGN = 0 
uint8 AP_INCREASE = 1
uint8 AP_DECREASE = 2
uint8 AP_SCALE_UP = 3
uint8 AP_SCALE_DOWN = 4
uint8 AP_ASSIGN_CTS = 5

uint8 assign_op

#-----------
# EXPRESSION
#-----------

string optimization
rosplan_knowledge_msgs/ExprComposite expr

#-----------
# INEQUALITY
#-----------

rosplan_knowledge_msgs/DomainInequality ineq
