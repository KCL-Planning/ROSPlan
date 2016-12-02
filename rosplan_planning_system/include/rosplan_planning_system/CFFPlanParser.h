#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "diagnostic_msgs/KeyValue.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "PlanningEnvironment.h"
#include "PlanParser.h"
#include "EsterelPlan.h"

#ifndef KCL_cff_plan_parser
#define KCL_cff_plan_parser

namespace KCL_rosplan {

	/* Plan Parsing class definition */
	class CFFPlanParser: public PlanParser
	{

	private:
		
		// ROS node handle.
		ros::NodeHandle* node_handle;
		
		// Knowledge base
		ros::ServiceClient update_knowledge_client;
		
		/* Contingent-FF assigned action IDs and branches */
		std::map<std::string,StrlNode*> cff_node_map;
		std::map<std::string,std::vector<StrlEdge*> > incoming_edge_map;

		/* operator observation and parameter mappings */
		std::map<std::string, std::string> operator_observation_map;
		std::map<std::string, std::vector<std::string> > operator_parameter_map;

		void toLowerCase(std::string &str);
		unsigned int split(const std::string &txt, std::vector<std::string> &strs, char ch);

		void parseDomain();
		void preparePDDLObservation(std::string &operator_name, std::vector<std::string> &parameters, StrlEdge &edge, bool isNegative);
		void preparePDDLConditions(std::string operator_name, std::vector<std::string> parameters, StrlNode &node, PlanningEnvironment &environment);

		void createNode(std::vector<std::string> &s, const std::string& operator_name, int node_id, PlanningEnvironment &environment, StrlNode& node);
		void createEdge(std::string &child_cffid, StrlNode &node, StrlEdge &edge);

	public:

		/* plan description in Esterel */
		std::vector<StrlNode*> plan_nodes;
		std::vector<StrlEdge*> plan_edges;

		/* constructor */
		CFFPlanParser(ros::NodeHandle &nh);

		/* service to parse plans */
		bool produceEsterel();

		/* virtual methods */
		void reset();
		void preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID);
		void generateFilter(PlanningEnvironment &environment);
	};
}
#endif
