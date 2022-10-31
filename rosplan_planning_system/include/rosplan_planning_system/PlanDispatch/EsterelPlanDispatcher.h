#ifndef KCL_esterel_dispatcher
#define KCL_esterel_dispatcher

#include <stdlib.h>
#include <map>
#include <iostream>
#include <string>
#include <boost/regex.hpp>
#include <boost/concept_check.hpp>

#include "PlanDispatcher.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"

#include "std_msgs/String.h"



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

		// plan status
		std::map<int,bool> edge_active;
		std::map<int,bool> action_dispatched;

		bool state_changed;
		bool finished_execution;

		//map real time dispatch for each node
		std::map<int,double> node_real_dispatch_time;

		/* plan graph publisher */
		bool printPlan();
		ros::Publisher plan_graph_publisher;

		ros::ServiceClient query_knowledge_client;
		ros::ServiceClient query_domain_client;

		bool display_edge_type_;

        /* dispatch flags */
        bool timeout_actions;
        double action_timeout_fraction;

	public:

		/* constructor */
		explicit EsterelPlanDispatcher(ros::NodeHandle& nh);
		~EsterelPlanDispatcher();

		void planCallback(const rosplan_dispatch_msgs::EsterelPlan plan);

		void reset() override;

		/* action dispatch methods */
		bool dispatchPlan(double missionStartTime, double planStartTime) override;

		/* action feedback methods */
		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) override;
	};
}

#endif
