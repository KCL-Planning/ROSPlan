#EsterelPlanNode message

byte ACTION_START = 0
byte ACTION_END   = 1
byte PLAN_START   = 2

byte node_type
int32 node_id
string name
rosplan_dispatch_msgs/ActionDispatch action

int32[] edges_out
int32[] edges_in
