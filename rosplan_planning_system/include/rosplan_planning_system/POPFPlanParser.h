/**
 * This file parses the output of popf and generates a list of ActionDispatch messages.
 * TODO Document
 */
#include <string>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "PlanningEnvironment.h"
#include "PlanParser.h"

#ifndef KCL_popf_plan_parser
#define KCL_popf_plan_parser

namespace KCL_rosplan {

	class POPFPlanParser: public PlanParser
	{
	private:

		/* plan knowledge filter */
		std::vector<std::string> filter_objects;
		std::vector<std::vector<std::string> > filter_attributes;

		/* post process plan */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params, PlanningEnvironment &environment);

	public:

		/* post process plan */
		double total_plan_duration;

		/* virtual methods */
		void reset();
		void preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID);
		void generateFilter(PlanningEnvironment &environment);

	};
} // close namespace

#endif
