/*
 * This file describes the class used to dispatch a Simple (sequential) PDDL plan.
 */

#ifndef KCL_simple_dispatcher
#define KCL_simple_dispatcher

#include "PlanDispatcher.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include <map>


namespace KCL_rosplan
{

	class SimplePlanDispatcher: public PlanDispatcher
	{
	private:

		// current plan and time plan was recevied
		rosplan_dispatch_msgs::CompletePlan current_plan;

		ros::ServiceClient queryKnowledgeClient;
		ros::ServiceClient queryDomainClient;

		/* current action to dispatch */
		int current_action;

	public:

		/* constructor */
		explicit SimplePlanDispatcher(ros::NodeHandle& nh);
		~SimplePlanDispatcher();

		void planCallback(const rosplan_dispatch_msgs::CompletePlan plan);

		void reset() override;

		bool dispatchPlan(double missionStartTime, double planStartTime) override;

		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) override;


	};
}

#endif
