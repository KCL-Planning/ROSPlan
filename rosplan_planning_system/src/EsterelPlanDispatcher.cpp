#include "rosplan_planning_system/EsterelPlanDispatcher.h"


namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	EsterelPlanDispatcher::EsterelPlanDispatcher(CFFPlanParser &parser)
		: action_id_offset(0)
	{
		ros::NodeHandle nh("~");
		nh.param("/rosplan/strl_file_path", strl_file, std::string("common/plan.strl"));

		plan_nodes = &(parser.plan_nodes);
		plan_edges = &(parser.plan_edges);

		current_action = 0;

		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		plan_graph_publisher = nh.advertise<std_msgs::String>("/kcl_rosplan/plan_graph", 1000, true);
	}

	EsterelPlanDispatcher::EsterelPlanDispatcher(POPFEsterelPlanParser &parser)
		: action_id_offset(0)
	{
		ros::NodeHandle nh("~");
		nh.param("strl_file_path", strl_file, std::string("common/plan.strl"));

		plan_nodes = &parser.plan_nodes;
		plan_edges = &parser.plan_edges;

		current_action = 0;

		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		plan_graph_publisher = nh.advertise<std_msgs::String>("/kcl_rosplan/plan_graph", 1000, true);
	}

	/*---------------*/
	/* public access */
	/*---------------*/

	int EsterelPlanDispatcher::getCurrentAction() {
		return current_action;
	}

	void EsterelPlanDispatcher::setCurrentAction(size_t freeActionID) {
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

	/*--------------*/
	/* read Esterel */
	/*--------------*/
	/*
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
	}*/

	/*-----------------*/
	/* action dispatch */
	/*-----------------*/

	/*
	 * Loop through and publish planned actions
	 */
	bool EsterelPlanDispatcher::dispatchPlan(const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actionList, double missionStart, double planStart) {

		ros::NodeHandle nh("~");
		ros::Rate loop_rate(10);
		
		std::string data_path;
		nh.param("/rosplan/data_path", data_path, std::string("common/"));

		// dispatch plan
		ROS_INFO("KCL: (EsterelPlanDispatcher) Dispatching plan");
		
		replan_requested = false;
		bool repeatAction = false;
		
		// initialise machine
		std::map<StrlEdge*,bool> edge_values;
		for(std::vector<StrlEdge*>::const_iterator ci = plan_edges->begin(); ci != plan_edges->end(); ci++)
			edge_values[*ci] = false;
		
		// query KMS for condition edges
		ROS_INFO("KCL: (EsterelPlanDispatcher) Initialise the external conditions.");

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
				ROS_ERROR("KCL: (EsterelPlanDispatcher) Query to KMS failed; no condition edges are true.");
			}
		}
		
		// begin execution
		bool finished_execution = false;
		bool state_changed = false;
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
				ROS_INFO("KCL: (EsterelPlanDispatcher) Plan has been cancelled!");
				break;
			}

			// for each module
			for(std::vector<StrlNode*>::const_iterator ci = plan_nodes->begin(); ci != plan_nodes->end(); ci++) {
				
				StrlNode* strl_node = *ci;
				
				// If at least one node is still executing we are not done yet.
				if (strl_node->dispatched && !strl_node->completed)
					finished_execution = false;
				
				if(!strl_node->dispatched && !strl_node->completed) {
				
					// check action edges
					bool activate_action = true;
					for (std::vector<StrlEdge*>::const_iterator ci = strl_node->input.begin(); ci != strl_node->input.end(); ++ci) {
						// Check if any action edge is unfinished
						if ((*ci)->signal_type == ACTION && !(*ci)->active) {
							activate_action = false;
						}
					}

					// query KMS for condition edges
					bool activate = true;
					if(activate_action) {
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
									ROS_ERROR("KCL: (EsterelPlanDispatcher) Query to KMS failed; no condition edges are true.");
								}
							}
						}
					}
					
					if(activate && activate_action) {

						finished_execution = false;
						state_changed = true;
						
						// activate action
						strl_node->dispatched = true;
						strl_node->completed = false;
						action_received[strl_node->node_id] = false;
						action_completed[strl_node->node_id] = false;
						rosplan_dispatch_msgs::ActionDispatch currentMessage = strl_node->dispatch_msg;
						
						currentMessage.action_id = currentMessage.action_id + action_id_offset;
						
						// dispatch action
						ROS_INFO("KCL: (EsterelPlanDispatcher) Dispatching action [%i, %s, %f, %f]", currentMessage.action_id, currentMessage.name.c_str(), (currentMessage.dispatch_time+planStart-missionStart), currentMessage.duration);
						action_publisher.publish(currentMessage);
						current_action = strl_node->node_id;
					}

				} else if(!strl_node->completed) {
					
					// check action completion
					if(action_completed[strl_node->node_id]) {

						strl_node->completed = true;
						finished_execution = false;
						state_changed = true;
						
						ROS_INFO("KCL: (EsterelPlanDispatcher) %i: action %s completed", strl_node->node_id, strl_node->node_name.c_str());

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
				// Reset edge values for unless they are from actions (completed action will always be complete)
				if ((*eit)->signal_type != ACTION) {
					edge_values[*eit] = false;
				}
			}

			if(state_changed)
				printPlan(data_path);
			ros::spinOnce();
			loop_rate.sleep();

			// cancel dispatch on replan
			if(replan_requested) {
				ROS_INFO("KCL: (EsterelPlanDispatcher) Replan requested");
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
			if((*it)->node_id == msg->action_id - action_id_offset)
				found = true;
		}
		// no matching action
		if(!found) return;

		ROS_INFO("KCL: (EsterelPlanDispatcher) Feedback received [%i, %s]", msg->action_id, msg->status.c_str());

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
		// nothing yet
	}

	/*--------------------*/
	/* Produce DOT graphs */
	/*--------------------*/

	bool EsterelPlanDispatcher::printPlan(const std::string& path) {

		// output file
		std::stringstream dest;

		dest << "digraph plan_" << action_id_offset << " {" << std::endl;

		// nodes
		for(std::vector<StrlNode*>::iterator nit = plan_nodes->begin(); nit!=plan_nodes->end(); nit++) {

			std::string name = (*nit)->node_name.substr(0, (*nit)->node_name.find(" "));
			dest <<  (*nit)->node_id << "[ label=\"" << name;
			if((*nit)->completed) dest << "\" style=\"fill: #77f; \"];" << std::endl;
			else if((*nit)->dispatched) dest << "\" style=\"fill: #7f7; \"];" << std::endl;
			else dest << "\" style=\"fill: #fff; \"];" << std::endl;
		}

		// edges
		for(std::vector<StrlEdge*>::iterator eit = plan_edges->begin(); eit!=plan_edges->end(); eit++) {
			for(int i=0; i<(*eit)->sources.size(); i++) {
				for(int j=0; j<(*eit)->sinks.size(); j++) {
					dest << "\"" << (*eit)->sources[i]->node_id << "\"" << " -> \"" << (*eit)->sinks[j]->node_id << "\";" << std::endl;
				}
			}
		}

		dest << "}" << std::endl;

		// publish on topic
		std_msgs::String msg;
		msg.data = dest.str();
		plan_graph_publisher.publish(msg);

		// write to file
		std::ofstream file;
		std::stringstream ss;
		ss << path << "/d3_viz/plan_" << action_id_offset << ".dot";
		file.open(ss.str().c_str());
		file << dest.str();
		file.close();
		
	}
} // close namespace
