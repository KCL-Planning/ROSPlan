#include <stdlib.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <algorithm>
#include <ctype.h>

#include "ros/ros.h"
#include "diagnostic_msgs/KeyValue.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "PlanningEnvironment.h"
#include "PlanParser.h"
#include "EsterelPlan.h"

#ifndef KCL_popf_esterel_parser
#define KCL_popf_esterel_parser

/**
 * This class describes the POPFEsterelPlanParser, which parses the output of popf and generates an Esterel plan.
 */
namespace KCL_rosplan {

	class POPFEsterelPlanParser: public PlanParser
	{
	private:

		// ROS node handle.
		ros::NodeHandle* node_handle;

		// Knowledge base
		ros::ServiceClient update_knowledge_client;

		// Configuration parameters
		std::set<std::string> concurrency_constraining_types;

		/* post process plan */
		void toLowerCase(std::string &str);
		void preparePDDLConditions(StrlNode &node, PlanningEnvironment &environment);
		
		/**
		 * Create an Estrel node based on the name of the action.
		 * @param action_name The name of the action.
		 * @param node_id The id that should be given to the node.
		 * @param environment The planning environment.
		 * @param node The node that is created based on @ref{action_name}.
		 * @param edge The edge that is created based on @ref{action_name}.
		 */
		void createNodeAndEdge(const std::string& action_name, double dispatchTime, double duration, int node_id, PlanningEnvironment &environment, StrlNode& node, StrlEdge& edge);

	public:

		/* plan description in Esterel */
		std::vector<StrlNode*> plan_nodes;
		std::vector<StrlEdge*> plan_edges;

		/* constructor */
		POPFEsterelPlanParser(ros::NodeHandle &nh);

		/* service to parse plans */
		bool produceEsterel();

		/* virtual methods */
		void reset();
		void preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID);
		void generateFilter(PlanningEnvironment &environment);
	};
} // close namespace

#endif
