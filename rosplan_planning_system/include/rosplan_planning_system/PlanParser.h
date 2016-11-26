/**
 * This file parses the output of a planner and generates a list of ActionDispatch messages.
 */
#include <string>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "PlanningEnvironment.h"

#ifndef KCL_plan_parser
#define KCL_plan_parser

namespace KCL_rosplan {

	class PlanParser
	{
	public:

		virtual void reset() =0;

		/* post process plan */
		virtual void preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) =0;
		std::vector<rosplan_dispatch_msgs::ActionDispatch> action_list;

		/* plan knowledge filter */
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> knowledge_filter;
		virtual void generateFilter(PlanningEnvironment &environment) =0;
	};
} // close namespace

#endif
