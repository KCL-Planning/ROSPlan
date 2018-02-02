/**
 * This file parses the output of popf and generates a list of ActionDispatch messages.
 */
#include "PlanParser.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "diagnostic_msgs/KeyValue.h"

#include <string>
#include <sstream>

#ifndef KCL_popf_plan_parser
#define KCL_popf_plan_parser

namespace KCL_rosplan {

	class POPFPlanParser: public PlanParser
	{
	private:

	protected:

		/* post process plan */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params);

		/* post process plan */
		double total_plan_duration;

		/* virtual methods */
		void reset();
		void preparePlan();
		void publishPlan();

	public:

		POPFPlanParser(ros::NodeHandle& nh);
		~POPFPlanParser();

		/* ROS interface */
		ros::Publisher plan_publisher;

	};
} // close namespace

#endif
