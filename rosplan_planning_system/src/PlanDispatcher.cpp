#include "rosplan_planning_system/PlanDispatcher.h"
#include <map>

namespace KCL_rosplan {

	int PlanDispatcher::getCurrentAction() {
		return current_action;
	}

	void PlanDispatcher::reset() {
		replan_requested = false;
		dispatch_paused = false;
		plan_cancelled = false;
		current_action = 0;
		action_received.clear();
		action_completed.clear();
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

		ROS_INFO("KCL: (PS) Dispatching plan");
		replan_requested = false;
		bool repeatAction = false;
		while (ros::ok() && actionList.size() > current_action) {

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
			rosplan_dispatch_msgs::ActionDispatch currentMessage = actionList[current_action];
			if((unsigned int)currentMessage.action_id != current_action)
				ROS_ERROR("KCL: (PS) Message action_id [%d] does not meet expected [%zu]", currentMessage.action_id, current_action);

			// loop while waiting for dispatch time
			if(!dispatch_on_completion) {
				double wait_period = 10.0;
				int wait_print = (int)(currentMessage.dispatch_time + planStart - ros::WallTime::now().toSec()) / wait_period;
				while (ros::ok() && ros::WallTime::now().toSec() < currentMessage.dispatch_time + planStart) {
					ros::spinOnce();
					loop_rate.sleep();
					double remaining = planStart + currentMessage.dispatch_time - ros::WallTime::now().toSec();
					if(wait_print > (int)remaining / wait_period) {
						ROS_INFO("KCL: (PS) Waiting %f before dispatching action: [%i, %s, %f, %f]",
								remaining,currentMessage.action_id, currentMessage.name.c_str(),
								 (currentMessage.dispatch_time+planStart-missionStart), currentMessage.duration);
						wait_print--;
					}
				}
			}

			if(!checkPreconditions(currentMessage)) {
				ROS_INFO("KCL: (PS) Preconditions not achieved [%i, %s]", currentMessage.action_id, currentMessage.name.c_str());

				// publish feedback (precondition false)
				rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = currentMessage.action_id;
				fb.status = "precondition false";
				action_feedback_pub.publish(fb);

				replan_requested = true;
			} else {

				// dispatch action
				ROS_INFO("KCL: (PS) Dispatching action [%i, %s, %f, %f]", currentMessage.action_id, currentMessage.name.c_str(), (currentMessage.dispatch_time+planStart-missionStart), currentMessage.duration);
				action_publisher.publish(currentMessage);
				double late_print = (ros::WallTime::now().toSec() - (currentMessage.dispatch_time + planStart));
				if(late_print>0.1) ROS_INFO("KCL: (PS) Action [%i] is %f second(s) late", currentMessage.action_id, late_print);

				// wait for action to complete
				if(!dispatch_concurrent) {
					int counter = 0;
					while (ros::ok() && !action_completed[current_action]) {
						ros::spinOnce();
						loop_rate.sleep();
						counter++;
						if (counter == 2000) {
							ROS_INFO("KCL: (PS) Action %i timed out now. Cancelling...", currentMessage.action_id);
							rosplan_dispatch_msgs::ActionDispatch cancelMessage;
							cancelMessage.action_id = currentMessage.action_id;
							cancelMessage.name = "cancel_action";
							action_publisher.publish(cancelMessage);
						}
					}
				}
			}

			// get ready for next action
			if(!repeatAction) current_action++;
			repeatAction = false;
			action_received[current_action] = false;
			action_completed[current_action] = false;

			// finish dispatch and replan
			if(replan_requested) return false;
		}
		return true;
	}

	bool PlanDispatcher::checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg) {

		// setup service call
		ros::NodeHandle nh;
		ros::ServiceClient queryKnowledgeClient = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		rosplan_knowledge_msgs::KnowledgeQueryService querySrv;

		std::map<std::string, std::vector<std::vector<std::string> > >::iterator oit;
		oit = environment.domain_operator_precondition_map.find(msg.name);
		if(oit==environment.domain_operator_precondition_map.end()) return false;

		// iterate through conditions
		std::vector<std::vector<std::string> >::iterator cit = oit->second.begin();
		for(; cit!=oit->second.end(); cit++) {
			
			rosplan_knowledge_msgs::KnowledgeItem condition;
			
			// set fact or function
			std::map<std::string,std::vector<std::string> >::iterator dit = environment.domain_predicates.find((*cit)[0]);
			if(dit!=environment.domain_predicates.end()) condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;

			dit = environment.domain_functions.find((*cit)[0]);
			if(dit!=environment.domain_functions.end()) condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;

			// populate parameters
			condition.attribute_name = (*cit)[0];
			int index = 1;
			std::vector<std::string>::iterator pit;
			for(pit=environment.domain_predicates[condition.attribute_name].begin();
					pit!=environment.domain_predicates[condition.attribute_name].end(); pit++) {
				// set parameter label to predicate label
				diagnostic_msgs::KeyValue param;
				param.key = *pit;
				// find label as it is in domain operator
				std::string conditionKey = (*cit)[index];
				index++;
				// set value
				std::vector<diagnostic_msgs::KeyValue>::iterator opit;
				for(opit = msg.parameters.begin(); opit!=msg.parameters.end(); opit++) {
					if(0==opit->key.compare(conditionKey)) {
						param.value = opit->value;
					}
				}
				condition.values.push_back(param);
			}
			querySrv.request.knowledge.push_back(condition);
		}

		// check conditions in knowledge base
		if (queryKnowledgeClient.call(querySrv)) {
			
			if(!querySrv.response.all_true) {
				std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kit;
				for(kit=querySrv.response.false_knowledge.begin(); kit != querySrv.response.false_knowledge.end(); kit++)
					ROS_INFO("KCL: (PS)        [%s]", kit->attribute_name.c_str());
			}
			return querySrv.response.all_true;

		} else {
			ROS_ERROR("KCL: (PS) Failed to call service /kcl_rosplan/query_knowledge_base");
		}
	}

	/*------------------*/
	/* general feedback */
	/*------------------*/

	/**
	 * listen to and process actionFeedback topic.
	 */
	void PlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		// create error if the action is unrecognised
		ROS_INFO("KCL: (PS) Feedback received [%i, %s]", msg->action_id, msg->status.c_str());
		if(current_action != (unsigned int)msg->action_id)
			ROS_ERROR("KCL: (PS) Unexpected action ID: %d; current action: %zu", msg->action_id, current_action);

		// action enabled
		if(!action_received[msg->action_id] && (0 == msg->status.compare("action enabled")))
			action_received[msg->action_id] = true;
		
		// more specific feedback
		actionFeedback(msg);

		// action completed (successfuly)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action achieved"))
			action_completed[msg->action_id] = true;

		// action completed (failed)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action failed")) {
			replan_requested = true;
			action_completed[msg->action_id] = true;
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
		// nothing yet...
	}
} // close namespace
