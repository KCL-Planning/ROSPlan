/**
 * This file defines the data used in the storing and dispatch of a plan.
 * This is intended for use with the popf temporal planner (v1.2), however
 * it should be flexible enough to allow for different planners.
 */
#include "planning_dispatch_msgs/ActionDispatch.h"
#include "planning_dispatch_msgs/ActionFeedback.h"

#ifndef KCL_dispatcher
#define KCL_dispatcher

namespace KCL_rosplan
{
	/* first free action ID */
	size_t freeActionID;

	/* action dispatch list (current plan) */
	std::vector<planning_dispatch_msgs::ActionDispatch> actionList;
	double totalPlanDuration;
	size_t currentAction;

	/* plan list (for remembering previous plans) */
	std::vector< std::vector<planning_dispatch_msgs::ActionDispatch> > planList;
	std::vector<size_t> planListLastAction;
	size_t planningAttempts;

	/* dispatch state */
	std::map<int,bool> actionReceived;
	std::map<int,bool> actionCompleted;
	bool replanRequested;
	bool dispatchPaused;

	/* action dispatch and feedback methods */
	ros::Publisher actionPublisher;
	ros::Subscriber feedbackSub;

	void publishAction(ros::NodeHandle nh);
	void feedbackCallback(const planning_dispatch_msgs::ActionFeedback::ConstPtr& msg);
}

#endif
