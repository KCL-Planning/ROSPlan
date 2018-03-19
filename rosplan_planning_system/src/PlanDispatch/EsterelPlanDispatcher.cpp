#include "rosplan_planning_system/PlanDispatch/EsterelPlanDispatcher.h"


namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	EsterelPlanDispatcher::EsterelPlanDispatcher(ros::NodeHandle& nh) {
		
		node_handle = &nh;

		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		query_domain_client = node_handle->serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/kcl_rosplan/get_domain_operator_details");

		std::string plan_graph_topic = "plan_graph";
		nh.getParam("plan_graph_topic", plan_graph_topic);
		plan_graph_publisher = node_handle->advertise<std_msgs::String>(plan_graph_topic, 1000, true);

		action_dispatch_topic = "action_dispatch";
		action_feedback_topic = "action_feedback";
		nh.getParam("action_dispatch_topic", action_dispatch_topic);
		nh.getParam("action_feedback_topic", action_feedback_topic);
		action_dispatch_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionDispatch>(action_dispatch_topic, 1, true);
		action_feedback_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionFeedback>(action_feedback_topic, 1, true);

		reset();
	}

	EsterelPlanDispatcher::~EsterelPlanDispatcher()
	{

	}

	void EsterelPlanDispatcher::reset() {
		replan_requested = false;
		dispatch_paused = false;
		plan_cancelled = false;
		action_received.clear();
		action_completed.clear();
		plan_recieved = false;
		finished_execution = true;
	}

	/*-------------------*/
	/* Plan subscription */
	/*-------------------*/

	void EsterelPlanDispatcher::planCallback(const rosplan_dispatch_msgs::EsterelPlan plan) {
		if(finished_execution) {
			ROS_INFO("KCL: (%s) Plan recieved.", ros::this_node::getName().c_str());
			plan_recieved = true;
			mission_start_time = ros::WallTime::now().toSec();
			current_plan = plan;
			printPlan();
		} else {
			ROS_INFO("KCL: (%s) Plan recieved, but current execution not yet finished.", ros::this_node::getName().c_str());
		}
	}

	/*--------------------*/
	/* Dispatch interface */
	/*--------------------*/

	/**
	 * plan dispatch service method (1) 
	 * dispatches plan as a service
	 * @returns True iff every action was dispatched and returned success.
	 */
	bool EsterelPlanDispatcher::dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
		if(!plan_recieved) return false;
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
	bool EsterelPlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {

		ROS_INFO("KCL: (%s) Dispatching plan.", ros::this_node::getName().c_str());

		ros::Rate loop_rate(10);
		replan_requested = false;
		plan_cancelled = false;
		
		// initialise machine
		initialise();

		// begin execution
		finished_execution = false;
		state_changed = false;
		while (ros::ok() && !finished_execution) {

			// loop while dispatch is paused
			while (ros::ok() && dispatch_paused) {
				ros::spinOnce();
				loop_rate.sleep();
			}

			// cancel plan
			if(plan_cancelled) {
				ROS_INFO("KCL: (%s) Plan cancelled.", ros::this_node::getName().c_str());
				break;
			}

			bool plan_started = false;
			finished_execution = true;
			state_changed = false;

			// for each node check completion, conditions, and dispatch
			for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
				
				rosplan_dispatch_msgs::EsterelPlanNode node = *ci;

				// activate plan start edges
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START && !plan_started) {

					// activate new edges
					std::vector<int>::const_iterator ci = node.edges_in.begin();
					ci = node.edges_out.begin();
					for(; ci != node.edges_out.end(); ci++) {
						edge_active[*ci] = true;
					}

					finished_execution = false;
					state_changed = true;
					plan_started = true;
				}
				
				// do not check actions for nodes which are not action nodes
				if(node.node_type != rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START && node.node_type != rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END)
					continue;

				// If at least one node is still executing we are not done yet
				if (action_dispatched[node.action.action_id] && !action_completed[node.action.action_id]) {
					finished_execution = false;
				}

				// check action edges
				bool edges_activate_action = true;
				std::vector<int>::iterator eit = node.edges_in.begin();
				for (; eit != node.edges_in.end(); ++eit) {
					if(!edge_active[(*eit)]) edges_activate_action = false;
				}
				if(!edges_activate_action) continue;

				// dispatch new action
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START && !action_dispatched[node.action.action_id]) {

					finished_execution = false;

					// query KMS for condition edges
					bool condition_activate_action = false;
					if(edges_activate_action) {			
						condition_activate_action = checkPreconditions(node.action);
					}

					if(condition_activate_action) {

						// activate action
						action_dispatched[node.action.action_id] = true;
						action_received[node.action.action_id] = false;
						action_completed[node.action.action_id] = false;

						// dispatch action
						ROS_INFO("KCL: (%s) Dispatching action [%i, %s, %f, %f]",
								ros::this_node::getName().c_str(), 
								node.action.action_id,
								node.action.name.c_str(),
								(node.action.dispatch_time+planStartTime-missionStartTime),
								node.action.duration);

						action_dispatch_publisher.publish(node.action);
						state_changed = true;

						// deactivate incoming edges
						std::vector<int>::const_iterator ci = node.edges_in.begin();
						for(; ci != node.edges_in.end(); ci++) {
							edge_active[*ci] = false;
						}

						// activate new edges
						ci = node.edges_out.begin();
						for(; ci != node.edges_out.end(); ci++) {
							edge_active[*ci] = true;
						}
					}
				}

				// handle completion of an action
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END && action_completed[node.action.action_id]) {

					ROS_INFO("KCL: (%s) %i: action %s completed",
							ros::this_node::getName().c_str(),
							node.action.action_id,
							node.action.name.c_str());

					finished_execution = false;
					state_changed = true;

					// deactivate incoming edges
					std::vector<int>::const_iterator ci = node.edges_in.begin();
					for(; ci != node.edges_in.end(); ci++) {
						edge_active[*ci] = false;
					}

					// activate new edges
					ci = node.edges_out.begin();
					for(; ci != node.edges_out.end(); ci++) {
						edge_active[*ci] = true;
					}
				}

			} // end loop (action nodes)

			ros::spinOnce();
			loop_rate.sleep();

			if(state_changed) printPlan();

			// cancel dispatch on replan
			if(replan_requested) {
				ROS_INFO("KCL: (%s) Replan requested.", ros::this_node::getName().c_str());
				reset();
				return false;
			}
		}

		ROS_INFO("KCL: (%s) Dispatch complete.", ros::this_node::getName().c_str());
		
		reset();
		return true;
	}

	void EsterelPlanDispatcher::initialise() {

		for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
			action_dispatched[ci->action.action_id] = false;
			action_received[ci->action.action_id] = false;
			action_completed[ci->action.action_id] = false;
		}

		for(std::vector<rosplan_dispatch_msgs::EsterelPlanEdge>::const_iterator ci = current_plan.edges.begin(); ci != current_plan.edges.end(); ci++) {
			edge_active[ci->edge_id] = false;
		}
	}


	/**
	 *	Returns true of the actions preconditions are true in the current state. Calls the Knowledge Base.
	 */
	bool EsterelPlanDispatcher::checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg) {

		// get domain opertor details
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = msg.name;
		if(!query_domain_client.call(srv)) {
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
		if (query_knowledge_client.call(querySrv)) {
			
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
	void EsterelPlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		ROS_INFO("KCL: (%s) Feedback received [%i, %s]", ros::this_node::getName().c_str(), msg->action_id, msg->status.c_str());

		// action enabled
		if(!action_received[msg->action_id] && (0 == msg->status.compare("action enabled"))) {
			action_received[msg->action_id] = true;
			state_changed = true;
		}

		// action completed (successfuly)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action achieved")) {

			// check action is part of current plan
			if(!action_received[msg->action_id]) {
				ROS_INFO("KCL: (%s) Action not yet dispatched, ignoring feedback", ros::this_node::getName().c_str());
			}
			action_completed[msg->action_id] = true;
			state_changed = true;
		}

		// action completed (failed)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action failed")) {
			replan_requested = true;
			action_completed[msg->action_id] = true;
		}
	}

	/*-------------------*/
	/* Produce DOT graph */
	/*-------------------*/

	bool EsterelPlanDispatcher::printPlan() {

		// output stream
		std::stringstream dest;

		dest << "digraph plan" << " {" << std::endl;

		// nodes
		for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::iterator nit = current_plan.nodes.begin(); nit!=current_plan.nodes.end(); nit++) {
			dest <<  nit->node_id << "[ label=\"" << nit->name;

			switch(nit->node_type) {
			case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START:
				if(action_received[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkolivegreen,fontcolor=white];" << std::endl;
				} else if(action_dispatched[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkgoldenrod2];" << std::endl;
				} else {
					dest << "\"];" << std::endl;
				}
				break;
			case rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END:
				if(action_completed[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkolivegreen,fontcolor=white];" << std::endl;
				} else if(action_dispatched[nit->action.action_id]) {
					dest << "\",style=filled,fillcolor=darkgoldenrod2];" << std::endl;
				} else {
					dest << "\"];" << std::endl;
				}
				break;
			case rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START:
				dest << "\",style=filled,fillcolor=black,fontcolor=white];" << std::endl;
				break;
			default:
				dest << "\"];" << std::endl;
				break;
			}
		}

		// edges
		for(std::vector<rosplan_dispatch_msgs::EsterelPlanEdge>::iterator eit = current_plan.edges.begin(); eit!=current_plan.edges.end(); eit++) {
			for(int j=0; j<eit->sink_ids.size(); j++) {
			for(int i=0; i<eit->source_ids.size(); i++) {

				dest << "\"" << eit->source_ids[i] << "\"" << " -> \"" << eit->sink_ids[j] << "\"";
				if(eit->duration_upper_bound == std::numeric_limits<double>::max()) {
					dest << " [ label=\"[" << eit->duration_lower_bound << ", " << "inf]\"";
				} else {
					dest << " [ label=\"[" << eit->duration_lower_bound << ", " << eit->duration_upper_bound << "]\"";
				}
				dest << " , penwidth=2"
						<< ((edge_active[eit->edge_id]) ? " , color=\"red\"" : " , color=\"black\"")
						<< "]" << std::endl;
			}};
		}

		dest << "}" << std::endl;

		// publish on topic
		std_msgs::String msg;
		msg.data = dest.str();
		plan_graph_publisher.publish(msg);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"rosplan_esterel_plan_dispatcher");
		ros::NodeHandle nh("~");

		KCL_rosplan::EsterelPlanDispatcher epd(nh);
	
		// subscribe to planner output
		std::string planTopic = "complete_plan";
		nh.getParam("plan_topic", planTopic);
		ros::Subscriber plan_sub = nh.subscribe(planTopic, 1, &KCL_rosplan::EsterelPlanDispatcher::planCallback, &epd);

		std::string feedbackTopic = "action_feedback";
		nh.getParam("action_feedback_topic", feedbackTopic);
		ros::Subscriber feedback_sub = nh.subscribe(feedbackTopic, 1000, &KCL_rosplan::EsterelPlanDispatcher::feedbackCallback, &epd);

		// start the plan parsing services
		ros::ServiceServer service1 = nh.advertiseService("dispatch_plan", &KCL_rosplan::PlanDispatcher::dispatchPlanService, dynamic_cast<KCL_rosplan::PlanDispatcher*>(&epd));
		ros::ServiceServer service2 = nh.advertiseService("cancel_dispatch", &KCL_rosplan::PlanDispatcher::cancelDispatchService, dynamic_cast<KCL_rosplan::PlanDispatcher*>(&epd));

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
