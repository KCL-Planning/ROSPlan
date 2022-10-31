#EsterelPlanEdge message

byte CONDITION_EDGE   = 0
byte START_END_ACTION_EDGE = 1
byte INTERFERENCE_EDGE = 2

byte edge_type
int32 edge_id
string edge_name
int32 signal_type
int32[] source_ids
int32[] sink_ids
float64 duration_lower_bound
float64 duration_upper_bound
