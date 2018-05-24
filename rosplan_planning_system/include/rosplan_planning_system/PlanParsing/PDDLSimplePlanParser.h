/**
 * This file parses standard PDDL output and generates a list of ActionDispatch messages.
 */
#include "PlanParser.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "diagnostic_msgs/KeyValue.h"

#include <string>
#include <sstream>

#ifndef KCL_pddl_simple_plan_parser
#define KCL_pddl_simple_plan_parser

namespace KCL_rosplan {

	class PDDLSimplePlanParser: public PlanParser
	{
	private:

		ros::ServiceClient get_operator_details_client;

	protected:

		/* post process plan */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params);

		/* virtual methods */
		void reset();
		void preparePlan();
		void publishPlan();

	public:

		PDDLSimplePlanParser(ros::NodeHandle& nh);
		~PDDLSimplePlanParser();

		/* ROS interface */
		ros::Publisher plan_publisher;

	};
} // close namespace

#endif
