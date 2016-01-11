#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "diagnostic_msgs/KeyValue.h"
#include "mongodb_store/message_store.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "PlanningEnvironment.h"
#include "PlanParser.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>

#ifndef KCL_cff_plan_parser
#define KCL_cff_plan_parser

namespace KCL_rosplan {

	/* Plan node definition */
	struct PlanNode
	{
		PlanNode(const int &nodeID, const std::string &actionName)
			: id(nodeID), action_name(actionName) {}

		PlanNode()
			: id(-1), action_name("GHOST_ACTION") {}
		
		int id;
		rosplan_dispatch_msgs::ActionDispatch dispatch_msg;
		std::string action_name;
		std::vector<int> inc_edges;
	};

	/* Plan Parsing class definition */
	class CFFPlanParser: public PlanParser
	{

	private:
		
		// Scene database
		mongodb_store::MessageStoreProxy message_store;

		// Knowledge base
		ros::ServiceClient update_knowledge_client;
		
	public:

		std::vector<PlanNode> plan;

		/* constructor */
		CFFPlanParser(ros::NodeHandle &nh);

		/* service to parse plans */
		bool printPlan(std::vector<PlanNode> &plan);
		bool produceEsterel(std::vector<PlanNode> &plan);

		/* virtual methods */
		void reset();
		void preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID);
		void generateFilter(PlanningEnvironment &environment);
	};
}
#endif
