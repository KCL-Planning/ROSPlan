/**
 * This file dispatches a plan. TODO document
 */
#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"

#ifndef KCL_dispatcher
#define KCL_dispatcher

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
		ros::Publisher action_publisher;
	};
}

#endif
