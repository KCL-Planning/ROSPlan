/*
 * This file describes the class used to dispatch a Simple (sequential) PDDL plan.
 */
#include "PlanDispatcher.h"

#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"

#include <map>

#ifndef KCL_simple_dispatcher
#define KCL_simple_dispatcher

namespace KCL_rosplan
{

	class SimplePlanDispatcher: public PlanDispatcher
	{
	private:

		// current plan and time plan was recevied
		rosplan_dispatch_msgs::CompletePlan current_plan;
		double mission_start_time;

		/* check preconditions are true */
		bool checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg);
		ros::ServiceClient queryKnowledgeClient;
		ros::ServiceClient queryDomainClient;

		/* current action to dispatch */
		int current_action;

	public:

		/* constructor */
		SimplePlanDispatcher(ros::NodeHandle& nh);
		~SimplePlanDispatcher();

		void planCallback(const rosplan_dispatch_msgs::CompletePlan plan);

		void reset();

		bool dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool dispatchPlan(double missionStartTime, double planStartTime);

		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
	};
}

#endif
