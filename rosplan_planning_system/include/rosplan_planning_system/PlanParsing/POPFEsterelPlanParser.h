#include <stdlib.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <algorithm>
#include "PlanParser.h"

#include "rosplan_planning_system/Plans/EsterelPlan.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "diagnostic_msgs/KeyValue.h"

#include <string>
#include <sstream>

#ifndef KCL_popf_esterel_parser
#define KCL_popf_esterel_parser

/**
 * This class describes the POPFEsterelPlanParser, which parses the output of popf and generates an Esterel plan.
 */
namespace KCL_rosplan {

	class POPFEsterelPlanParser: public PlanParser
	{
	private:

		/* plan description in Esterel */
		std::vector<StrlNode*> plan_nodes;
		std::vector<StrlEdge*> plan_edges;

		/* post process plan */
		void preparePDDLConditions(StrlNode &node);
		void createNodeAndEdge(const std::string& action_name, double dispatchTime, double duration, int node_id, StrlNode& node, StrlEdge& edge);

	protected:

		/* post process plan */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params);

		/* virtual methods */
		void reset();
		void preparePlan();
		void publishPlan();

	public:

		/* constructor */
		POPFEsterelPlanParser(ros::NodeHandle &nh);

		/* ROS interface */
		ros::Publisher plan_publisher;
	};
} // close namespace

#endif
