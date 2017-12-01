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
	}

	/*-------------------*/
	/* Plan subscription */
	/*-------------------*/

	void EsterelPlanDispatcher::planCallback(const rosplan_dispatch_msgs::EsterelPlan plan) {
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
		
		// initialise machine
		initialise();

		// begin execution
		finished_execution = false;
		state_changed = false;
		while (ros::ok() && !finished_execution) {

			finished_execution = true;
			state_changed = false;

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

			// for each node check completion, conditions, and dispatch
			for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
				
				rosplan_dispatch_msgs::EsterelPlanNode node = *ci;
				
				// If at least one node is still executing we are not done yet.
				if (action_dispatched[node.action.action_id] && !action_completed[node.action.action_id])
					finished_execution = false;
				
				if(!action_dispatched[node.action.action_id]) {
				
					// check action edges
					bool activate_action = false;
					std::vector<int>::iterator eit = node.edges_in.begin();
					for (; eit != node.edges_in.end(); ++eit) {
						if(edge_active[(*eit)]) activate_action = true;
					}

					// query KMS for condition edges
					bool activate = false;
					if(node.edges_in.size()==0 || activate_action) {
						activate = checkPreconditions(node.action);
					}
					
					if(activate && (node.edges_in.size()==0 || activate_action)) {

						finished_execution = false;
						state_changed = true;
						
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
					}

				} else if(action_completed[node.action.action_id]) {

					// reset node
					if (node.edges_in.size() > 0) {
						action_dispatched[node.action.action_id] = false;
						action_received[node.action.action_id] = false;
						action_completed[node.action.action_id] = false;
					}

				}

			} // end loop (nodes)

			// deactivate all edges
			std::vector<rosplan_dispatch_msgs::EsterelPlanEdge>::const_iterator ci = current_plan.edges.begin();
			for(; ci != current_plan.edges.end(); ci++) {
				edge_active[ci->edge_id] = false;
			}

			ros::spinOnce();
			loop_rate.sleep();

			if(state_changed) printPlan();

			// cancel dispatch on replan
			if(replan_requested) {
				ROS_INFO("KCL: (%s) Replan requested.", ros::this_node::getName().c_str());
				return false;
			}
		}
		
		return true;
	}

	void EsterelPlanDispatcher::initialise() {

		// query KMS for condition edges
		ROS_INFO("KCL: (%s) Initialise the external conditions.", ros::this_node::getName().c_str());

		for(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ci = current_plan.nodes.begin(); ci != current_plan.nodes.end(); ci++) {
			action_dispatched[ci->action.action_id] = false;
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
		if(!action_received[msg->action_id] && (0 == msg->status.compare("action enabled")))
			action_received[msg->action_id] = true;

		// action completed (successfuly)
		if(!action_completed[msg->action_id] && 0 == msg->status.compare("action achieved")) {
			action_completed[msg->action_id] = true;

			// activate new edges
			std::vector<int>::iterator eit = current_plan.nodes[msg->action_id].edges_out.begin();
			for (; eit != current_plan.nodes[msg->action_id].edges_out.end(); ++eit) {
				edge_active[(*eit)] = true;
			}
			
			finished_execution = false;
			state_changed = true;

			ROS_INFO("KCL: (%s) %i: action %s completed", ros::this_node::getName().c_str(), msg->action_id, current_plan.nodes[msg->action_id].action.name.c_str());

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
/*
		// output stream
		std::stringstream dest;

		dest << "digraph plan" << " {" << std::endl;

		// nodes
		for(std::vector<StrlNode*>::iterator nit = plan_nodes->begin(); nit!=plan_nodes->end(); nit++) {
			std::string name = (*nit)->node_name;//.substr(0, (*nit)->node_name.find(" "));
			dest <<  (*nit)->node_id << "[ label=\"" << name;
			if((*nit)->completed) dest << "\" style=\"fill: #77f; \"];" << std::endl;
			else if((*nit)->dispatched) dest << "\" style=\"fill: #7f7; \"];" << std::endl;
			else dest << "\" style=\"fill: #fff; \"];" << std::endl;
		}

		// edges
		for(std::vector<StrlEdge*>::iterator eit = plan_edges->begin(); eit!=plan_edges->end(); eit++) {

			std::stringstream label;
			if((*eit)->signal_type == CONDITION && (*eit)->external_conditions.size()>0) {
				std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator cit = (*eit)->external_conditions.begin();
				if(cit->is_negative)
					label << "(not ";
				label << "(" << cit->attribute_name;
				for(int i=0; i<cit->values.size(); i++) {
					label << " " << cit->values[i].value;
				}
				label << ")";
				if(cit->is_negative)
					label << ")";
			}

			for(int i=0; i<(*eit)->sources.size(); i++) {
				for(int j=0; j<(*eit)->sinks.size(); j++) {
					if((*eit)->signal_type == CONDITION && (*eit)->external_conditions.size()>0) {
						// add edge with label of external condition
						dest << "\"" << (*eit)->sources[i]->node_id << "\"" << " -> \"" << (*eit)->sinks[j]->node_id << "\" [ label=\"" << label.str() << "\" ];" << std::endl;
					} else if((*eit)->signal_type == CONDITION && (*eit)->external_conditions.size()==0) {
						// add edge with label of condition, references by edge name
						dest << "\"" << (*eit)->sources[i]->node_id << "\"" << " -> \"" << (*eit)->sinks[j]->node_id << "\" [ label=\"" << (*eit)->edge_name << "\" ];" << std::endl;
					} else {

						// add edge without any label
						dest << "\"" << (*eit)->sources[i]->node_id << "\"" << " -> \"" << (*eit)->sinks[j]->node_id << "\"" << std::endl;
					}
				}
			}
		}

		dest << "}" << std::endl;

		// publish on topic
		std_msgs::String msg;
		msg.data = dest.str();
		plan_graph_publisher.publish(msg);
*/
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
		ros::Subscriber feedback_sub = nh.subscribe(feedbackTopic, 1, &KCL_rosplan::EsterelPlanDispatcher::feedbackCallback, &epd);

		// start the plan parsing services
		ros::ServiceServer service1 = nh.advertiseService("dispatch_plan", &KCL_rosplan::PlanDispatcher::dispatchPlanService, dynamic_cast<KCL_rosplan::PlanDispatcher*>(&epd));

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
