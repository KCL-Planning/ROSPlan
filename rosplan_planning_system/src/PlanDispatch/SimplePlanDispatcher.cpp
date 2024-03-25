
#include <rosplan_planning_system/PlanDispatch/SimplePlanDispatcher.h>

#include "rosplan_planning_system/PlanDispatch/SimplePlanDispatcher.h"

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	SimplePlanDispatcher::SimplePlanDispatcher(ros::NodeHandle& nh) : PlanDispatcher(nh)  {

		node_handle = &nh;

		// knowledge base services
		std::stringstream ss;
		ss << "/" << kb_ << "/query_state";
		queryKnowledgeClient = node_handle->serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>(ss.str());
		ss.str("");

		reset();
	}

	SimplePlanDispatcher::~SimplePlanDispatcher()
	{

	}

	void SimplePlanDispatcher::reset() {
		PlanDispatcher::reset();
		current_action = 0;
	}

	/*-------------------*/
	/* Plan subscription */
	/*-------------------*/

	void SimplePlanDispatcher::planCallback(const rosplan_dispatch_msgs::CompletePlan plan) {
        
		ROS_INFO("KCL: (%s) Plan received.", ros::this_node::getName().c_str());
		ROS_INFO("KCL: (%s) Is plan empty?: %d", ros::this_node::getName().c_str(), plan.plan.size() == 0);
        
		plan_received = true;
		mission_start_time = ros::WallTime::now().toSec();
		current_plan = plan;
	}

	/*-----------------*/
	/* action dispatch */
	/*-----------------*/

	/*
	 * Loop through and publish planned actions
	 */
	bool SimplePlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {

		ROS_INFO("KCL: (%s) Dispatching plan", ros::this_node::getName().c_str());
		ROS_DEBUG("KCL: (%s) Num actions: %zu", ros::this_node::getName().c_str(), current_plan.plan.size());
		ROS_DEBUG("KCL: (%s) Current action: %d", ros::this_node::getName().c_str(), current_action);

		ros::Rate loop_rate(10);
		replan_requested = false;

        while (ros::ok() && current_plan.plan.size() > current_action) {

			// loop while dispatch is paused
			while (ros::ok() && dispatch_paused) {
				ros::spinOnce();
				loop_rate.sleep();
			}

			// cancel plan
			if(plan_cancelled) {
				break;
			}

			// get next action
			rosplan_dispatch_msgs::ActionDispatch currentMessage = current_plan.plan[current_action];

			// check action preconditions
			if(!checkPreconditions(currentMessage)) {

				ROS_INFO("KCL: (%s) Preconditions not achieved [%i, %s]", ros::this_node::getName().c_str(), currentMessage.action_id, currentMessage.name.c_str());

				// publish feedback (precondition false)
				rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = currentMessage.action_id;
				fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_PRECONDITION_FALSE;
				publishFeedback(fb);

				replan_requested = true;

			} else {

			    std:: string params = "(";
			    for (size_t i = 0; i < currentMessage.parameters.size(); ++i) {
			        if (i > 0) params += ", ";
			        params += currentMessage.parameters[i].value;
			    }
			    params += ")";
				// dispatch action
				ROS_INFO("KCL: (%s) Dispatching action [%i, %s%s, %f, %f]",
						ros::this_node::getName().c_str(), 
						currentMessage.action_id,
						currentMessage.name.c_str(),
						params.c_str(),
						(currentMessage.dispatch_time+planStartTime-missionStartTime),
						currentMessage.duration);

				action_dispatch_publisher.publish(currentMessage);
                // publish feedback (action dispatched)
                rosplan_dispatch_msgs::ActionFeedback fb;
                fb.action_id = currentMessage.action_id;
                fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_DISPATCHED_TO_GOAL_STATE;
                publishFeedback(fb);

				double late_print = (ros::WallTime::now().toSec() - (currentMessage.dispatch_time+planStartTime));
				if(late_print>0.1) {
					ROS_INFO("KCL: (%s) Action [%i] is %f second(s) late", ros::this_node::getName().c_str(), currentMessage.action_id, late_print);
				}

				// wait for action to complete
				while (ros::ok() && !action_completed[current_action]) {
					ros::spinOnce();
					loop_rate.sleep();
				}
			}

			// get ready for next action
			current_action++;
			action_received[current_action] = false;
			action_completed[current_action] = false;

			// finish dispatch and replan
			if(replan_requested) return false;
		}

		ROS_INFO("KCL: (%s) Dispatch complete.", ros::this_node::getName().c_str());
		return true;
	}



	/*------------------*/
	/* general feedback */
	/*------------------*/

	/**
	 * listen to and process actionFeedback topic.
	 */
	void SimplePlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		// create error if the action is unrecognised
		ROS_INFO("KCL: (%s) Feedback received [%i, %d]", ros::this_node::getName().c_str(), msg->action_id, msg->status);
		if(current_action != (unsigned int)msg->action_id)
			ROS_ERROR("KCL: (%s) Unexpected action ID: %d; current action: %d", ros::this_node::getName().c_str(), msg->action_id, current_action);

		// action enabled
		if(!action_received[msg->action_id] && msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_ENABLED)
			action_received[msg->action_id] = true;

		// action completed (successfuly)
		if(!action_completed[msg->action_id] && msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE)
			action_completed[msg->action_id] = true;

		// action completed (failed)
		if(!action_completed[msg->action_id] && msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_FAILED) {
			replan_requested = true;
			action_completed[msg->action_id] = true;
		}
	}


} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc,argv,"rosplan_simple_plan_dispatcher");
    ros::NodeHandle nh("~");

    KCL_rosplan::SimplePlanDispatcher spd(nh);

    // subscribe to planner output
    std::string planTopic = "complete_plan";
    nh.getParam("plan_topic", planTopic);
    ros::Subscriber plan_sub = nh.subscribe(planTopic, 1, &KCL_rosplan::SimplePlanDispatcher::planCallback, &spd);

    std::string feedbackTopic = "action_feedback";
    nh.getParam("action_feedback_topic", feedbackTopic);
    ros::Subscriber feedback_sub = nh.subscribe(feedbackTopic, 1000, &KCL_rosplan::SimplePlanDispatcher::feedbackCallback, &spd);

    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}
