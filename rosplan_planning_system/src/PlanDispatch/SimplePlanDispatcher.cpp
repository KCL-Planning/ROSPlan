#include "rosplan_planning_system/PlanDispatch/SimplePlanDispatcher.h"

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	SimplePlanDispatcher::SimplePlanDispatcher(ros::NodeHandle& nh) {
		
		node_handle = &nh;

		queryKnowledgeClient = node_handle->serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		queryDomainClient = node_handle->serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/kcl_rosplan/get_domain_operator_details");

		action_dispatch_topic = "action_dispatch";
		action_feedback_topic = "action_feedback";
		nh.getParam("action_dispatch_topic", action_dispatch_topic);
		nh.getParam("action_feedback_topic", action_feedback_topic);
		action_dispatch_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionDispatch>(action_dispatch_topic, 1, true);
		action_feedback_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionFeedback>(action_feedback_topic, 1, true);

		reset();
	}

	SimplePlanDispatcher::~SimplePlanDispatcher()
	{

	}

	void SimplePlanDispatcher::reset() {
		replan_requested = false;
		dispatch_paused = false;
		plan_cancelled = false;
		action_received.clear();
		action_completed.clear();
		plan_recieved = false;
		current_action = 0;
	}

	/*-------------------*/
	/* Plan subscription */
	/*-------------------*/

	void SimplePlanDispatcher::planCallback(const rosplan_dispatch_msgs::CompletePlan plan) {
		ROS_INFO("KCL: (%s) Plan recieved.", ros::this_node::getName().c_str());
		plan_recieved = true;
		mission_start_time = ros::WallTime::now().toSec();
		current_plan = plan;
	}

	/*--------------------*/
	/* Dispatch interface */
	/*--------------------*/

	/**
	 * plan dispatch service method (1) 
	 * dispatches plan as a service
	 * @returns True iff every action was dispatched and returned success.
	 */
	bool SimplePlanDispatcher::dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
		bool success = dispatchPlan(mission_start_time,ros::WallTime::now().toSec());
		reset();
		return success;
	}

	/*-----------------*/
	/* action dispatch */
	/*-----------------*/

	/*
	 * Loop through and publish planned actions
	 */
	bool SimplePlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {

		ROS_INFO("KCL: (%s) Dispatching plan", ros::this_node::getName().c_str());

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
				fb.status = "precondition false";
				action_feedback_publisher.publish(fb);

				replan_requested = true;

			} else {

				// dispatch action
				ROS_INFO("KCL: (%s) Dispatching action [%i, %s, %f, %f]",
						ros::this_node::getName().c_str(), 
						currentMessage.action_id,
						currentMessage.name.c_str(),
						(currentMessage.dispatch_time+planStartTime-missionStartTime),
						currentMessage.duration);

				action_dispatch_publisher.publish(currentMessage);

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
		return true;
	}

	/**
	 *	Returns true of the actions preconditions are true in the current state. Calls the Knowledge Base.
	 */
	bool SimplePlanDispatcher::checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg) {

		// get domain opertor details
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = msg.name;
		if(!queryDomainClient.call(srv)) {
			ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s", ros::this_node::getName().c_str(), msg.name.c_str());
			return false;
		}

		// setup service call
		rosplan_knowledge_msgs::DomainOperator op = srv.response.op;
		rosplan_knowledge_msgs::KnowledgeQueryService querySrv;

		// iterate through conditions
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit = op.at_start_simple_condition.begin();
		for(; cit!=op.at_start_simple_condition.end(); cit++) {

			// create condition
			rosplan_knowledge_msgs::KnowledgeItem condition;
			condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			condition.attribute_name = cit->name;

			// populate parameters
			for(int i=0; i<cit->typed_parameters.size(); i++) {

				// set parameter label to predicate label
				diagnostic_msgs::KeyValue param;
				param.key = cit->typed_parameters[i].key;

				// search for correct operator parameter value
				for(int j=0; j<msg.parameters.size() && j<op.formula.typed_parameters.size(); j++) {
					if(op.formula.typed_parameters[j].key == cit->typed_parameters[i].value) {
						param.value = msg.parameters[j].value;
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
					ROS_INFO("KCL: (%s) Precondition not achieved: %s", ros::this_node::getName().c_str(), kit->attribute_name.c_str());
			}
			return querySrv.response.all_true;

		} else {
			ROS_ERROR("KCL: (%s) Failed to call service /kcl_rosplan/query_knowledge_base", ros::this_node::getName().c_str());
		}
	}

	/*------------------*/
	/* general feedback */
	/*------------------*/

	/**
	 * listen to and process actionFeedback topic.
	 */
	void SimplePlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		// create error if the action is unrecognised
		ROS_INFO("KCL: (%s) Feedback received [%i, %s]", ros::this_node::getName().c_str(), msg->action_id, msg->status.c_str());
		if(current_action != (unsigned int)msg->action_id)
			ROS_ERROR("KCL: (%s) Unexpected action ID: %d; current action: %d", ros::this_node::getName().c_str(), msg->action_id, current_action);

		// action enabled
		if(!action_received[msg->action_id] && (0 == msg->status.compare("action enabled")))
			action_received[msg->action_id] = true;

		// action completed (successfuly)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action achieved"))
			action_completed[msg->action_id] = true;

		// action completed (failed)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action failed")) {
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
		ros::Subscriber feedback_sub = nh.subscribe(feedbackTopic, 1, &KCL_rosplan::SimplePlanDispatcher::feedbackCallback, &spd);

		// start the plan parsing services
		ros::ServiceServer service1 = nh.advertiseService("dispatch_plan", &KCL_rosplan::PlanDispatcher::dispatchPlanService, dynamic_cast<KCL_rosplan::PlanDispatcher*>(&spd));
		ros::ServiceServer service2 = nh.advertiseService("cancel_dispatch", &KCL_rosplan::PlanDispatcher::cancelDispatchService, dynamic_cast<KCL_rosplan::PlanDispatcher*>(&spd));

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
