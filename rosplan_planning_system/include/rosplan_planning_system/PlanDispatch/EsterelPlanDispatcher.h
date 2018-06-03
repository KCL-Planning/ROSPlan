#include <stdlib.h> 
#include <map>
#include <iostream>
#include <string>
#include <boost/regex.hpp>
#include <boost/concept_check.hpp>

#include "PlanDispatcher.h"

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

		/**
		 * @returns node ID associated with action ID if it exists, -1 otherwise
		 */
		int getNodeFromActionID(int action_id, bool end_node) {
			std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin();
			for(; ci != current_plan.nodes.end(); ci++) {
				if(ci->action.action_id == action_id) {
					if(!end_node && ci->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START)
						return ci->node_id;
					else if(end_node && ci->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END)
						return ci->node_id;
				}
			}
			return -1;
		}

		// esterel plan methods
		void initialise();
	
		// current plan and time plan was recevied
		rosplan_dispatch_msgs::EsterelPlan current_plan;
		double mission_start_time;

		// plan status
		std::map<int,bool> edge_active;
		std::map<int,bool> action_dispatched;

		bool state_changed;
		bool finished_execution;

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
