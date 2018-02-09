#include "PlanParser.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include "diagnostic_msgs/KeyValue.h"

#include <stdlib.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <algorithm>

#ifndef KCL_pddl_esterel_plan_parser
#define KCL_pddl_esterel_plan_parser

/**
 * This class describes the POPFEsterelPlanParser, which parses the output of popf and generates an Esterel plan.
 */
namespace KCL_rosplan {

	class PDDLEsterelPlanParser: public PlanParser
	{
	private:

		/**
		 * @returns True if the two domain formula parameters match completely.
		 */
		bool domainFormulaMatches(rosplan_knowledge_msgs::DomainFormula& a, rosplan_knowledge_msgs::DomainFormula& b) {
			if(b.name != a.name) return false;
			if(b.typed_parameters.size() != a.typed_parameters.size()) return false;
			for(size_t i=0; i<a.typed_parameters.size(); i++) {
				if(a.typed_parameters[i].value != b.typed_parameters[i].value) {
					return false;
				}
			}
			return true;
		}

		/* plan description in Esterel */
		rosplan_dispatch_msgs::EsterelPlan last_plan;
		std::map<int,rosplan_knowledge_msgs::DomainOperator> action_details;

	protected:

		/* post process plan */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params);
		void createGraph();
		bool createEdge(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ait, rosplan_knowledge_msgs::DomainFormula& condition, bool negative_condition);
		bool satisfiesPrecondition(rosplan_knowledge_msgs::DomainFormula& condition, const rosplan_dispatch_msgs::EsterelPlanNode& node, bool negative_condition);

		/* virtual methods */
		void reset();
		void preparePlan();
		void publishPlan();

	public:

		/* constructor */
		PDDLEsterelPlanParser(ros::NodeHandle &nh);
		~PDDLEsterelPlanParser();

		/* ROS interface */
		ros::Publisher plan_publisher;
	};
} // close namespace

#endif
