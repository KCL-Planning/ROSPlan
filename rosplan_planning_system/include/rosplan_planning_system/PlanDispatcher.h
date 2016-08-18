/**
 * This file describes the class that dispatches a plan.
 */
#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "PlanningEnvironment.h"

#ifndef KCL_dispatcher
#define KCL_dispatcher

namespace KCL_rosplan
{

	class PlanDispatcher
	{
	protected:

		/* action dispatch list (current plan) */
		size_t current_action;

		/* dispatch state */
		std::map<int,bool> action_received;
		std::map<int,bool> action_completed;

	public:

		/* knowledge */
		PlanningEnvironment environment;

		/* dispatch modes */
		bool dispatch_paused;
		bool plan_cancelled;
		bool replan_requested;

		/* access */
		virtual int getCurrentAction() =0;
		virtual void setCurrentAction(size_t freeActionID) =0;
		virtual void reset() =0;

		/* action dispatch methods */
		virtual bool dispatchPlan(const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actionList, double missionStart, double planStart) =0;

		/* action feedback methods */
		virtual void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) =0;
		virtual void actionFeedback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) =0;

		/* ROS interface */
		ros::Publisher action_publisher;
		ros::Publisher action_feedback_pub;
	};
}

#endif
