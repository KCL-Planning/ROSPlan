#include <stdlib.h> 
#include <map>
#include <iostream>
#include <string>
#include <boost/regex.hpp>
#include <boost/concept_check.hpp>

#include "PlanDispatcher.h"

#include "rosplan_planning_system/Plans/EsterelPlan.h"

#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"

#include "std_msgs/String.h"

#ifndef KCL_esterel_dispatcher
#define KCL_esterel_dispatcher

namespace KCL_rosplan
{
	class EsterelPlanDispatcher: public PlanDispatcher
	{
	private:
	
		// current plan and time plan was recevied
		rosplan_dispatch_msgs::EsterelPlan current_plan;
		double mission_start_time;
		std::vector<StrlNode*>* plan_nodes;
		std::vector<StrlEdge*>* plan_edges;

		/* plan graph publisher */
		bool printPlan();
		ros::Publisher plan_graph_publisher;

		/* check preconditions are true */
		bool checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg);
		ros::ServiceClient query_knowledge_client;
		ros::ServiceClient query_domain_client;

	public:

		/* constructor */
		EsterelPlanDispatcher(ros::NodeHandle& nh);
		~EsterelPlanDispatcher();

		void planCallback(const rosplan_dispatch_msgs::EsterelPlan plan);

		void reset();

		/* action dispatch methods */
		bool dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool dispatchPlan(double missionStartTime, double planStartTime);

		/* action feedback methods */
		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
	};
}

#endif
