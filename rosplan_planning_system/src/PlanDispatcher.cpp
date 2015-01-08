/**
 * This file contains the parsing of the ActionFeedback topic.
 * The feedback from actions can be verbose, and it is here that the data is parsed.
 * 
 * However: the discovery of new information, and consequent reasoning, should be
 * passed through knowledge messages (see the Plan Filter.)  In this way reasoning does
 * not need to wait for actions to be terminated, but can continue in parallel and
 * possibly interrupt action execution.
 */
#include "PlanDispatcher.h"

namespace KCL_rosplan {

	int PlanDispatcher::getCurrentAction() {
		return current_action;
	}

	/*-----------------*/
	/* action dispatch */
	/*-----------------*/

	/*
	 * Loop through and publish planned actions
	 */
	bool PlanDispatcher::dispatchPlan(const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actionList, double missionStart, double planStart) {

		ros::NodeHandle nh("~");
		ros::Rate loop_rate(10);

		bool repeatAction = false;

		while (ros::ok() && actionList.size() > current_action) {

			// get next action
			rosplan_dispatch_msgs::ActionDispatch currentMessage = actionList[current_action];
			if((unsigned int)currentMessage.action_id != current_action)
				ROS_INFO("KCL: ERROR message action_id [%d] does not meet expected [%zu]", currentMessage.action_id, current_action);

			// loop while waiting for dispatch time
			double wait_period = 10.0;
			int wait_print = (int)(currentMessage.dispatch_time + planStart - ros::WallTime::now().toSec()) / wait_period;
			while (ros::ok() && ros::WallTime::now().toSec() < currentMessage.dispatch_time + planStart) {
				ros::spinOnce();
				loop_rate.sleep();
				double remaining = planStart + currentMessage.dispatch_time - ros::WallTime::now().toSec();
				if(wait_print > (int)remaining / wait_period) {
					ROS_INFO("KCL: Waiting %f before dispatching action: [%i, %s, %f, %f]",
							remaining,currentMessage.action_id, currentMessage.name.c_str(),
							 (currentMessage.dispatch_time+planStart-missionStart), currentMessage.duration);
					wait_print--;
				}
			}

			// dispatch action
			ROS_INFO("KCL: Dispatching action [%i, %s, %f, %f]", currentMessage.action_id, currentMessage.name.c_str(), (currentMessage.dispatch_time+planStart-missionStart), currentMessage.duration);
			actionPublisher.publish(currentMessage);
			double late_print = (ros::WallTime::now().toSec() - (currentMessage.dispatch_time + planStart));
			if(late_print>0.1) ROS_INFO("KCL: Action [%i] is %f second(s) late", currentMessage.action_id, late_print);

			// callback and sleep
			int counter = 0;
			while (ros::ok() && !actionCompleted[current_action]) {
				ros::spinOnce();
				loop_rate.sleep();
				counter++;
				if (counter == 2000) {
					ROS_INFO("KCL: Action %i timeout now. Cancelling...", currentMessage.action_id);
					rosplan_dispatch_msgs::ActionDispatch cancelMessage;
					cancelMessage.action_id = currentMessage.action_id;
					cancelMessage.name = "cancel_action";
					actionPublisher.publish(cancelMessage);
				}
			}

			// get ready for next action
			if(!repeatAction) current_action++;
			repeatAction = false;
			actionReceived[current_action] = false;
			actionCompleted[current_action] = false;

			// finish dispatch and replan
			if(replanRequested) return false;
		}
		return true;
	}

	/*------------------*/
	/* general feedback */
	/*------------------*/

	/**
	 * listen to and process actionFeedback topic.
	 */
	void PlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		// create error if the action is unrecognised
		ROS_INFO("KCL: Feedback received [%i,%s]", msg->action_id, msg->status.c_str());
		if(current_action != (unsigned int)msg->action_id)
			ROS_INFO("KCL: Unexpected action ID: %d; current action: %zu", msg->action_id, current_action);

		// action enabled
		if(!actionReceived[msg->action_id] && (0 == msg->status.compare("action enabled")))
			actionReceived[msg->action_id] = true;
		
		// more specific feedback
		actionFeedback(msg);

		// action completed (successfuly)
		if(!actionCompleted[msg->action_id] && 0 == msg->status.compare("action achieved"))
			actionCompleted[msg->action_id] = true;

		// action completed (failed)
		if(!actionCompleted[msg->action_id] && 0 == msg->status.compare("action failed")) {
			actionCompleted[msg->action_id] = true;
		}
	}

	/*---------------------------*/
	/* Specific action responses */
	/*---------------------------*/

	/**
	 * processes single action feedback message.
	 * This method serves as the hook for defining more interesting behaviour on action feedback.
	 */
	void PlanDispatcher::actionFeedback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {
		ROS_INFO("KCL: Feedback received [%i,%s]", msg->action_id, msg->status.c_str());
	}
} // close namespace
