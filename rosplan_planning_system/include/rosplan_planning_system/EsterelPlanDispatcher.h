#include <stdlib.h> 
#include <map>
#include <iostream>
#include <string>
#include <boost/regex.hpp>
#include <boost/concept_check.hpp>

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "std_msgs/String.h"

#include "PlanDispatcher.h"
#include "EsterelPlan.h"
#include "CFFPlanParser.h"
#include "POPFEsterelPlanParser.h"

#ifndef KCL_esterel_dispatcher
#define KCL_esterel_dispatcher

namespace KCL_rosplan
{
	class EsterelPlanDispatcher: public PlanDispatcher
	{
	private:

		bool printPlan(const std::string& path);

		/* mapping PDDL conditions and esterel inputs */
		ros::ServiceClient query_knowledge_client;

		/* plan description from parser */
		std::vector<StrlNode*> * plan_nodes;
		std::vector<StrlEdge*> * plan_edges;

		/* plan description from Esterel file */
		std::string strl_file;
		bool readEsterelFile(std::string strlFile);
		
		/* action offset */
		int action_id_offset;

		/* plan graph publisher */
		ros::Publisher plan_graph_publisher;
		
		/* mapping between the node ids and the node */
		std::map<int, StrlNode*> strl_node_mapping;

	public:

		/* constructor */
		EsterelPlanDispatcher(CFFPlanParser &parser);
		EsterelPlanDispatcher(POPFEsterelPlanParser &parser);

		/* access */
		int getCurrentAction();
		void setCurrentAction(size_t freeActionID);
		void reset();

		/* action dispatch methods */
		bool dispatchPlan(const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actionList, double missionStart, double planStart);

		/* action feedback methods */
		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
		void actionFeedback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
	};
}

#endif
