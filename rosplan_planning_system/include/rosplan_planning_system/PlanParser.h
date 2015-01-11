/**
 * This file parses the output of popf and generates a list of ActionDispatch messages.
 * TODO Document
 */
#include <string>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "PlanningEnvironment.h"

namespace KCL_rosplan {

	class PlanParser
	{
	private:

		/* plan knowledge filter */
		std::vector<std::string> filter_objects;
		std::vector<std::vector<std::string> > filter_attributes;

		/* post process plan */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params, PlanningEnvironment &environment);

	public:

		void reset();

		/* plan knowledge filter */
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> knowledge_filter;
		void generateFilter(PlanningEnvironment &environment);

		/* post process plan */
		void preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID);
		std::vector<rosplan_dispatch_msgs::ActionDispatch> action_list;
		double total_plan_duration;

	};
} // close namespace
