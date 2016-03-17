#include "rosplan_dispatch_msgs/ActionDispatch.h"

#include <ostream>

#ifndef KCL_esterel_plan
#define KCL_esterel_plan

namespace KCL_rosplan
{

	enum SignalType { ACTION, CONDITION, TIME };

	/* Plan edge definition */
	struct StrlEdge
	{
		SignalType signal_type;
		std::string edge_name;
		std::vector<std::string> sources;
		std::vector<std::string> sinks;
		bool active;
	};
	
	

	/* Plan node definition */
	struct StrlNode
	{
		std::string node_name;
		int node_id;
		std::vector<std::string> input;
		std::vector<std::string> output;

		std::vector<bool> await_input;
		bool dispatched;
		bool completed;

		rosplan_dispatch_msgs::ActionDispatch dispatch_msg;
	};
}

std::ostream& operator<<(std::ostream& os, const KCL_rosplan::StrlEdge& edge);
std::ostream& operator<<(std::ostream& os, const KCL_rosplan::StrlNode& edge);

#endif
