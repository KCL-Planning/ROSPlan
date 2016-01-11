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

	EsterelPlanDispatcher::EsterelPlanDispatcher() {
		ros::NodeHandle nh("~");
		nh.param("strl_file_path", strl_file, std::string("common/plan.strl"));
	}

	/*---------------*/
	/* public access */
	/*---------------*/

	int EsterelPlanDispatcher::getCurrentAction() {
		return current_action;
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
							edge.edge_name = (*iter)[1];
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
		ROS_INFO("KCL: (PS) Parsing STRL file: %s", strl_file.c_str());
		readEsterelFile(strl_file);

		// dispatch plan
		ROS_INFO("KCL: (PS) Dispatching plan");
		replan_requested = false;
		bool repeatAction = false;
		
		// initialise machine
		std::map<std::string,bool> edge_values;
		std::map<std::string,StrlEdge>::iterator eit = plan_edges.begin();
		for(; eit!=plan_edges.end(); eit++)
			edge_values[eit->second.edge_name] = false;

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

			// for each module
			std::map<std::string,StrlNode>::iterator it = plan_description.begin();
			for(; it!=plan_description.end(); it++) {
	
				if(!it->second.dispatched) {

					// activate waiting nodes
					bool activate = true;
					for(int i=0;i<it->second.await_input.size();i++) {
						if(!plan_edges[it->second.input[i]].active)
							activate = false;
					}
					if(activate) {

						// activate action
						it->second.dispatched = true;
						current_action = atoi(it->second.node_name.substr(0,6).c_str());
						rosplan_dispatch_msgs::ActionDispatch currentMessage = actionList[current_action];
						action_received[current_action] = false;
						action_completed[current_action] = false;

						// dispatch action
						ROS_INFO("KCL: (PS) Dispatching action [%i, %s, %f, %f]", currentMessage.action_id, currentMessage.name.c_str(), (currentMessage.dispatch_time+planStart-missionStart), currentMessage.duration);
						action_publisher.publish(currentMessage);
						double late_print = (ros::WallTime::now().toSec() - (currentMessage.dispatch_time + planStart));
						if(late_print>0.1) ROS_INFO("KCL: (PS) Action [%i] is %f second(s) late", currentMessage.action_id, late_print);

					}

				} else if(!it->second.completed) {

					// check action completion
					int actionID = atoi(it->second.node_name.substr(0,6).c_str());
					if(action_completed[current_action]) {
						it->second.completed = true;

						// emit output edges
						for(int i=0;i<it->second.output.size();i++)
							edge_values[it->second.output[i]] = true;
					}
				}

				ros::spinOnce();
				loop_rate.sleep();
			}

			// copy new edge values
			std::map<std::string,StrlEdge>::iterator eit = plan_edges.begin();
			for(; eit!=plan_edges.end(); eit++) {
				eit->second.active = edge_values[eit->second.edge_name];
				edge_values[eit->second.edge_name] = false;
			}

			if(replan_requested) return false;
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
	void EsterelPlanDispatcher::actionFeedback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {
		// nothing yet...
	}
} // close namespace
