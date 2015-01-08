/**
 * This file defines the data used in the storing and dispatch of a plan.
 * This is intended for use with the popf temporal planner (v1.2), however
 * it should be flexible enough to allow for different planners.
 */

#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"

#ifndef KCL_plan_dispatcher
#define KCL_plan_dispatcher

namespace KCL_rosplan
{

	class PlanDispatcher
	{
	private:

		/* action dispatch list (current plan) */
		double totalPlanDuration;
		size_t current_action;

		/* dispatch state */
		std::map<int,bool> actionReceived;
		std::map<int,bool> actionCompleted;
		bool replanRequested;
		bool dispatchPaused;

	public:

		int getCurrentAction();

		/* action dispatch methods */
		bool dispatchPlan(const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actionList, double missionStart, double planStart);

		/* action feedback methods */
		void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);
		void actionFeedback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg);

		/* ROS interface */
		ros::Publisher actionPublisher;
	};
}

#endif
