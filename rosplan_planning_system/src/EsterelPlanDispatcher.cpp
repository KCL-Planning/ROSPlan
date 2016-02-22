#include "rosplan_planning_system/EsterelPlanDispatcher.h"
#include <stdlib.h> 
#include <map>
#include <iostream>
#include <string>
#include <boost/regex.hpp>

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	EsterelPlanDispatcher::EsterelPlanDispatcher(CFFPlanParser &parser)
		: action_id_offset(0)
	{
		current_action = 0;
		cff_pp = &parser;
		ros::NodeHandle nh("~");
		nh.param("strl_file_path", strl_file, std::string("common/plan.strl"));
		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
	}

	/*---------------*/
	/* public access */
	/*---------------*/

	int EsterelPlanDispatcher::getCurrentAction() {
		return current_action;
	}

	void EsterelPlanDispatcher::setCurrentAction(size_t freeActionID) {
		//current_action = freeActionID;
		action_id_offset = freeActionID;
	}

	void EsterelPlanDispatcher::reset() {
		replan_requested = false;
		dispatch_paused = false;
		plan_cancelled = false;
		current_action = 0;
		action_received.clear();
		action_completed.clear();
	}

	/*---------------*/
	/* parse esterel */
	/*---------------*/

	void EsterelPlanDispatcher::preparePDDLCondition(std::string edgeName) {

		// regex through the conditions
		boost::regex e_conditions("-([^-]+)");
		boost::sregex_iterator iter(edgeName.begin(), edgeName.end(), e_conditions);
		boost::sregex_iterator end;
		
		// prepare the attribute knowledge item
		rosplan_knowledge_msgs::KnowledgeItem item;
		item.knowledge_type = 1;
		item.attribute_name = (*iter)[1];

		// fetch attribute signiature
		std::map<std::string,std::vector<std::string> >::iterator it;
		it = environment.domain_predicates.find(item.attribute_name);
		if (it != environment.domain_predicates.end()) {
			int param = 0;
			// set parameters
			for(; iter != end; ++iter ) {
				if(param<=it->second.size()) {
					std::cout << "error reading condition edge!" << std::endl;
					return;
				}
				diagnostic_msgs::KeyValue pair;
				pair.key = it->second[param];
				pair.value = (*iter)[1];
				param++;
			}
		} else {
			std::cout << "error reading condition edge (2)!" << std::endl;
			return;
		}
		
		condition_mapping[edgeName] = item;

	}

	bool EsterelPlanDispatcher::readEsterelFile(std::string strlFile) {

		// open file
		std::ifstream planfile;
		std::cout << strlFile.c_str() << std::endl;
		planfile.open(strlFile.c_str());
		
		int curr, next; 
		std::string line;
		
		double planDuration;
		double expectedPlanDuration = 0;
		plan_description.clear();
		plan_edges.clear();

		bool readingModule = false;
		std::string currentNode;

		if(!planfile.is_open())
			return false;

		while(!planfile.eof()) {

			getline(planfile, line);

			boost::regex e_whitespace("^[\f\r\t\v]*\%.*");
			if (boost::regex_match(line,e_whitespace)) {
				// line is a comment
				continue;
			}

			boost::smatch what;
			boost::regex e_parameters("[\\s]*([^,;]+)[,;]");
			if(readingModule) {
  				// "end module"
				boost::regex e_module_end("^end module[\\s]*");
				if (boost::regex_match(line,e_module_end))
					readingModule = false;
				// "input [i0], [i1], [i2];"
				boost::regex e_input("^([\\s]*input[\\s]+).*$");
				if (boost::regex_match(line,what,e_input)) {
					std::string params = line.substr(what[1].length());
					boost::sregex_iterator iter(params.begin(), params.end(), e_parameters);
					boost::sregex_iterator end;
					for(; iter != end; ++iter ) {
						if(plan_edges.find((*iter)[1]) == plan_edges.end()) {
							StrlEdge edge;
							edge.signal_type = 0;
							edge.edge_name = (*iter)[1];
							if(edge.edge_name.length() > 3 && edge.edge_name.substr(0,4)=="cond") {
								edge.signal_type = 1;
								preparePDDLCondition(edge.edge_name);
							}
							plan_edges[(*iter)[1]] = edge;
						}
						plan_description[currentNode].input.push_back((*iter)[1]);
						plan_description[currentNode].await_input.push_back(true);
					}
				}
				// "output [i0], [i1], [i2];"
				boost::regex e_output("^([\\s]*output[\\s]+).*$");
				if (boost::regex_match(line,what,e_output)) {
					std::string params = line.substr(what[1].length());
					boost::sregex_iterator iter(params.begin(), params.end(), e_parameters);
					boost::sregex_iterator end;
					for(; iter != end; ++iter ) {
						if(plan_edges.find((*iter)[1]) == plan_edges.end()) {
							StrlEdge edge;
							edge.signal_type = 0;
							edge.edge_name = (*iter)[1];
							plan_edges[(*iter)[1]] = edge;
						}
						plan_description[currentNode].output.push_back((*iter)[1]);
					}
				}
			} else {
  				// "module [module_name]:"
				boost::regex e_module("^[\\s]*module[\\s+]([^:]+):.*");
				if (boost::regex_match(line,what,e_module)) {
					StrlNode node;
					node.node_name = what[1];
					plan_description[node.node_name] = node;
					currentNode = node.node_name;
					node.dispatched = false;
					node.completed = false;
					readingModule = true;
				}
			}
		}
		planfile.close();
	}

	/*-----------------*/
	/* action dispatch */
	/*-----------------*/

	/*
	 * Loop through and publish planned actions
	 */
	bool EsterelPlanDispatcher::dispatchPlan(const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actionList, double missionStart, double planStart) {

		ros::NodeHandle nh("~");
		ros::Rate loop_rate(10);

		// parse strl file
		ROS_INFO("KCL: (EsterelPlanDispatcher) %d Parsing STRL file: %s", action_id_offset, strl_file.c_str());
		if (!readEsterelFile(strl_file))
		{
			ROS_ERROR("KCL: (EsterelPlanDispatcher) %d Failed to read STRL file: %s", action_id_offset, strl_file.c_str());
			return false;
		}

		// dispatch plan
		ROS_INFO("KCL: (EsterelPlanDispatcher) Dispatching plan %d", current_action);
		replan_requested = false;
		bool repeatAction = false;
		
		// initialise machine
		std::map<std::string,bool> edge_values;
		std::map<std::string,StrlEdge>::iterator eit = plan_edges.begin();
		for(; eit!=plan_edges.end(); eit++)
			edge_values[eit->second.edge_name] = false;
		
		ROS_INFO("KCL: (EsterelPlanDispatcher) %d Complete plan:", action_id_offset);
		
		/*
		for (std::vector<PlanNode>::const_iterator ci = cff_pp->plan.begin(); ci != cff_pp->plan.end(); ++ci)
		{
			const PlanNode& plane_node = *ci;
			const rosplan_dispatch_msgs::ActionDispatch& action_dispatch = plane_node.dispatch_msg;
			ROS_INFO("KCL: (EsterelPlanDispatcher) %d Action [%i, %s, %f, %f]", action_id_offset, action_dispatch.action_id, action_dispatch.name.c_str(), (action_dispatch.dispatch_time+planStart-missionStart), action_dispatch.duration);
		}
		
		for (std::map<std::string,StrlNode>::const_iterator ci = plan_description.begin(); ci != plan_description.end(); ++ci)
		{
			const std::string& name = (*ci).first;
			const StrlNode& node = (*ci).second;
			
			ROS_INFO("KCL: (EsterelPlanDispatcher) %d Plan node %s (=%s) ", action_id_offset, name.c_str(), node.node_name.c_str());
			for (std::vector<std::string>::const_iterator ci = node.input.begin(); ci != node.input.end(); ++ci)
			{
				ROS_INFO("\tIN %s", (*ci).c_str());
			}
			for (std::vector<std::string>::const_iterator ci = node.output.begin(); ci != node.output.end(); ++ci)
			{
				ROS_INFO("\tOUT %s", (*ci).c_str());
			}
		}
		*/
		
		bool finished_execution = false;
		while (ros::ok() && !finished_execution) {

			finished_execution = true;
			// loop while dispatch is paused
			while (ros::ok() && dispatch_paused) {
				ros::spinOnce();
				loop_rate.sleep();
			}

			// cancel plan
			if(plan_cancelled) {
				ROS_INFO("KCL: (EsterelPlanDispatcher) %d Plan has been cancelled!", action_id_offset);
				break;
			}

			// for each module
			std::map<std::string,StrlNode>::iterator it = plan_description.begin();
			for(; it!=plan_description.end(); it++) {
				
				StrlNode& strl_node = it->second;
				
				// If at least one node is still executing we are not done yet.
				if (strl_node.dispatched && !strl_node.completed)
					finished_execution = false;
				
				if(!strl_node.dispatched) {
				
					// activate waiting nodes
					bool activate = true;
					for(int i=0;i<strl_node.await_input.size();i++) {
						
						if(!plan_edges[strl_node.input[i]].active)
						{
							activate = false;
						}
					}
					
					if(activate) {

						finished_execution = false;
						
						// activate action
						strl_node.dispatched = true;
						current_action = atoi(strl_node.node_name.substr(std::string("action").length()).c_str());
						//rosplan_dispatch_msgs::ActionDispatch currentMessage = actionList[current_action];
						rosplan_dispatch_msgs::ActionDispatch currentMessage = cff_pp->plan[current_action].dispatch_msg;
						action_received[current_action] = false;
						action_completed[current_action] = false;
						currentMessage.action_id = currentMessage.action_id + action_id_offset;

						// dispatch action
						ROS_INFO("KCL: (EsterelPlanDispatcher) %d Dispatching action [%i, %s, %f, %f]; current_ation = %d", action_id_offset, currentMessage.action_id, currentMessage.name.c_str(), (currentMessage.dispatch_time+planStart-missionStart), currentMessage.duration, current_action);
						action_publisher.publish(currentMessage);
						double late_print = (ros::WallTime::now().toSec() - (currentMessage.dispatch_time + planStart));
						if(late_print>0.1) ROS_INFO("KCL: (EsterelPlanDispatcher) %d Action [%i] is %f second(s) late", action_id_offset, currentMessage.action_id, late_print);
					}
				} else if(!strl_node.completed) {
					
					// check action completion
					int actionID = atoi(strl_node.node_name.substr(0,6).c_str());
					//std::cout << "Action ID: " << actionID << std::endl;
					if(action_completed[current_action]) {
						strl_node.completed = true;
						finished_execution = false;
						
						ROS_INFO("KCL: (EsterelPlanDispatcher) %d The action %s is completed!", action_id_offset, strl_node.node_name.c_str());
						//std::cout << "It is :)" << std::endl;

						// emit output edges
						for(int i=0;i<strl_node.output.size();i++) {
							ROS_INFO("KCL: (EsterelPlanDispatcher) %d Activate the edge %s", action_id_offset, strl_node.output[i].c_str());
							edge_values[strl_node.output[i]] = true;
						}
					}
				}

				printPlan();

				ros::spinOnce();
				loop_rate.sleep();
			}

			// query KMS for condition edges
			std::map<std::string,rosplan_knowledge_msgs::KnowledgeItem>::iterator kit = condition_mapping.begin();
			rosplan_knowledge_msgs::KnowledgeQueryService querySrv;
			for(; kit!=condition_mapping.end(); kit++) {
				querySrv.request.knowledge.clear();
				querySrv.request.knowledge.push_back(kit->second);
				if (query_knowledge_client.call(querySrv)) {
					edge_values[it->first] = querySrv.response.all_true;
				}
			}
			
			// copy new edge values
			std::map<std::string,StrlEdge>::iterator eit = plan_edges.begin();
			for(; eit!=plan_edges.end(); eit++) {
				eit->second.active = edge_values[eit->second.edge_name];
				edge_values[eit->second.edge_name] = false;
			}

			if(replan_requested) {
				ROS_INFO("KCL: (EsterelPlanDispatcher) %d Replan requested!", action_id_offset);
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

		// create error if the action is unrecognised
		ROS_INFO("KCL: (EsterelPlanDispatcher) %d Feedback received [%i, %s]", action_id_offset, msg->action_id, msg->status.c_str());
		if(current_action + action_id_offset != (unsigned int)msg->action_id) {
			ROS_WARN("KCL: (EsterelPlanDispatcher) %d Unexpected action ID: %d; current action: %zu", action_id_offset, msg->action_id, current_action + action_id_offset);
			return;
		} else {
			ROS_INFO("KCL: (EsterelPlanDispatcher) %d Expected action ID: %d!", action_id_offset, msg->action_id);
		}

		// action enabled
		int normalised_action_id = msg->action_id - action_id_offset;
		if(!action_received[normalised_action_id] && (0 == msg->status.compare("action enabled")))
			action_received[normalised_action_id] = true;
		
		// more specific feedback
		actionFeedback(msg);

		// action completed (successfuly)
		if(!action_completed[normalised_action_id] && 0 == msg->status.compare("action achieved"))
			action_completed[normalised_action_id] = true;

		// action completed (failed)
		if(!action_completed[normalised_action_id] && 0 == msg->status.compare("action failed")) {
			replan_requested = true;
			action_completed[normalised_action_id] = true;
		}
	}

	/*---------------------------*/
	/* Specific action responses */
	/*---------------------------*/

	/**
	 * processes single action feedback message.
	 * This method serves as the hook for defining more interesting behaviour on action feedback.
	 */
	void EsterelPlanDispatcher::actionFeedback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {
		// nothing yet...
	}

	/*--------------------*/
	/* Produce DOT graphs */
	/*--------------------*/

	bool EsterelPlanDispatcher::printPlan() {

		// output file
		std::ofstream dest;
		dest.open("plan.dot");

		dest << "digraph plan {" << std::endl;

		// nodes
		for(int i=0;i<cff_pp->plan.size();i++) {
			dest <<  cff_pp->plan[i].id << "[ label=\"" << cff_pp->plan[i].action_name;
			if(action_completed[cff_pp->plan[i].id]) dest << "\" style=\"fill: #77f; \"];" << std::endl;
			else if(action_received[cff_pp->plan[i].id]) dest << "\" style=\"fill: #7f7; \"];" << std::endl;
			else dest << "\" style=\"fill: #fff; \"];" << std::endl;
		}

		// edges
		for(int i=0;i<cff_pp->plan.size();i++) {
			for(int j=0;j<cff_pp->plan[i].inc_edges.size();j++) {
				if(cff_pp->plan[i].inc_edges[j] >= 0)
					dest <<  cff_pp->plan[i].inc_edges[j] << " -> " << cff_pp->plan[i].id << ";" << std::endl;
			}
		}

		dest << "}" << std::endl;
		dest.close();

	}
} // close namespace
