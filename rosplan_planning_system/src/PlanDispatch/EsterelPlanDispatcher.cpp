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
		std::map<StrlEdge*,bool> edge_values;
		for(std::vector<StrlEdge*>::const_iterator ci = plan_edges->begin(); ci != plan_edges->end(); ci++) {
			edge_values[*ci] = false;
		}

		// query KMS for condition edges
		ROS_INFO("KCL: (%s) Initialise the external conditions.", ros::this_node::getName().c_str());

		for (std::vector<StrlEdge*>::const_iterator ci = plan_edges->begin(); ci != plan_edges->end(); ++ci) {
			StrlEdge* edge = *ci;
			
			if (edge->external_conditions.empty()) {
				continue;
			}
			
			rosplan_knowledge_msgs::KnowledgeQueryService querySrv;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = edge->external_conditions.begin(); ci != edge->external_conditions.end(); ++ci) {
				const rosplan_knowledge_msgs::KnowledgeItem& knowledge_item = *ci;
				querySrv.request.knowledge.push_back(knowledge_item);
			}
			
			// Check if all external conditions have been satisfied.
			if (query_knowledge_client.call(querySrv)) {
				if (querySrv.response.all_true) {
					edge->active = true;
				}
			} else {
				ROS_ERROR("KCL: (%s) Query to KMS failed; no condition edges are made true.", ros::this_node::getName().c_str());
			}
		}
		
		// begin execution
		bool state_changed = false;
		bool finished_execution = false;
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
			for(std::vector<StrlNode*>::const_iterator ci = plan_nodes->begin(); ci != plan_nodes->end(); ci++) {
				
				StrlNode* strl_node = *ci;
				
				// If at least one node is still executing we are not done yet.
				if (strl_node->dispatched && !strl_node->completed)
					finished_execution = false;
				
				if(!strl_node->dispatched) {
				
					// check action edges
					int action_edge_count = 0;
					bool activate_action = false;
					for (std::vector<StrlEdge*>::const_iterator ci = strl_node->input.begin(); ci != strl_node->input.end(); ++ci) {
						if ((*ci)->signal_type == ACTION) {
							action_edge_count++;
							if ((*ci)->active) {
								activate_action = true;
							}
						}
					}

					// query KMS for condition edges
					bool activate = true;
					if(action_edge_count==0 || activate_action) {
						for (std::vector<StrlEdge*>::const_iterator ci = strl_node->input.begin(); ci != strl_node->input.end(); ++ci) {
							StrlEdge* edge = *ci;
							if ((*ci)->signal_type == CONDITION && !edge->external_conditions.empty()) {
								// Check if all external conditions have been satisfied.
								rosplan_knowledge_msgs::KnowledgeQueryService querySrv;
								querySrv.request.knowledge = edge->external_conditions;
								if (query_knowledge_client.call(querySrv)) {
									edge_values[edge] = querySrv.response.all_true;
									if (!querySrv.response.all_true) {
										activate = false;
										break;
									}
								} else {
									ROS_ERROR("KCL: (%s) Query to KMS failed.", ros::this_node::getName().c_str());
								}
							}
						}
					}
					
					if(activate && (action_edge_count==0 || activate_action)) {

						finished_execution = false;
						state_changed = true;
						
						// activate action
						strl_node->dispatched = true;
						strl_node->completed = false;
						action_received[strl_node->node_id] = false;
						action_completed[strl_node->node_id] = false;
						rosplan_dispatch_msgs::ActionDispatch currentMessage = strl_node->dispatch_msg;
						
						currentMessage.action_id = currentMessage.action_id;
						
						// dispatch action
						ROS_INFO("KCL: (%s) Dispatching action [%i, %s, %f, %f]",
								ros::this_node::getName().c_str(), 
								currentMessage.action_id,
								currentMessage.name.c_str(),
								(currentMessage.dispatch_time+planStartTime-missionStartTime),
								currentMessage.duration);

						action_dispatch_publisher.publish(currentMessage);
					}

				} else if(!strl_node->completed) {
					
					// check action completion
					if(action_completed[strl_node->node_id]) {

						strl_node->completed = true;
						finished_execution = false;
						state_changed = true;
						
						ROS_INFO("KCL: (%s) %i: action %s completed",
								ros::this_node::getName().c_str(),
								strl_node->node_id,
								strl_node->node_name.c_str());

						// emit output edges in next loop
						for(int i=0;i<strl_node->output.size();i++) {
							edge_values[strl_node->output[i]] = true;
						}
						
						// reset node
						if (!strl_node->input.empty()) {
							strl_node->dispatched = false;
						}
					}

				} // end if(!dispatched)

			} // end loop (nodes)
			
			// copy new edge values for next loop
			for(std::vector<StrlEdge*>::iterator eit = plan_edges->begin(); eit != plan_edges->end(); eit++) {
				(*eit)->active = edge_values[*eit];
				edge_values[*eit] = false;
			}

			if(state_changed) printPlan();
			ros::spinOnce();
			loop_rate.sleep();

			// cancel dispatch on replan
			if(replan_requested) {
				ROS_INFO("KCL: (%s) Replan requested.", ros::this_node::getName().c_str());
				return false;
			}
		}
		
		return true;
	}

	/*------------------*/
	/* general feedback */
	/*------------------*/

	/**
	 * listen to and process actionFeedback topic.
	 */
	void EsterelPlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

		// find action
		bool found = false;
		for(std::vector<StrlNode*>::iterator it = plan_nodes->begin(); it!=plan_nodes->end(); it++) {
			if((*it)->node_id == msg->action_id)
				found = true;
		}
		// no matching action
		if(!found) return;

		ROS_INFO("KCL: (%s) Feedback received [%i, %s]", ros::this_node::getName().c_str(), msg->action_id, msg->status.c_str());

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

	/*-------------------*/
	/* Produce DOT graph */
	/*-------------------*/

	bool EsterelPlanDispatcher::printPlan() {

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
						dest << "\"" << (*eit)->sources[i]->node_id << "\"" << " -> \"" << (*eit)->sinks[j]->node_id << "\" [ label=\"" << label.str()/*(*eit)->edge_name*/ << "\" ];" << std::endl;
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
