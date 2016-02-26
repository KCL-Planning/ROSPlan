#include "rosplan_planning_system/CFFPlanParser.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>

/* implementation of rosplan_interface_mapping::CFFPlanParser.h */
namespace KCL_rosplan {

	/* constructor */
	CFFPlanParser::CFFPlanParser(ros::NodeHandle &nh) : node_handle(&nh), message_store(nh)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	}

	void CFFPlanParser::reset() {
		plan_nodes.clear();
		plan_edges.clear();
		edge_conditions.clear();
	}

	void CFFPlanParser::generateFilter(PlanningEnvironment &environment) {
		// do nothing yet
	}

	void CFFPlanParser::toLowerCase(std::string &str) {
		const int length = str.length();
		for(int i=0; i < length; ++i) {
			str[i] = std::tolower(str[i]);
		}
	}

	/*------------*/
	/* parse PDDL */
	/*------------*/

	/**
	 * parse a PDDL condition
	 */
	void CFFPlanParser::preparePDDLConditions(StrlNode &node, PlanningEnvironment &environment) {

		// find action conditions
		std::map<std::string, std::vector<std::vector<std::string> > >::iterator oit;
		oit = environment.domain_operator_precondition_map.find(node.dispatch_msg.name);
		if(oit==environment.domain_operator_precondition_map.end()) {
std::cout << "action precondition map entry not found" << std::endl;
			return;
		}

		// iterate through conditions
		std::vector<std::vector<std::string> >::iterator cit = oit->second.begin();
		for(; cit!=oit->second.end(); cit++) {
			
			rosplan_knowledge_msgs::KnowledgeItem condition;
			
			// set fact or function
			std::map<std::string,std::vector<std::string> >::iterator dit = environment.domain_predicates.find((*cit)[0]);
			if(dit!=environment.domain_predicates.end()) condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;

			dit = environment.domain_functions.find((*cit)[0]);
			if(dit!=environment.domain_functions.end()) condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;

			// create edge name
			condition.attribute_name = (*cit)[0];
			std::stringstream ss;
			ss << condition.attribute_name;

			// populate parameters
			int index = 1;
			std::vector<std::string>::iterator pit = environment.domain_predicates[condition.attribute_name].begin();
			for(; pit!=environment.domain_predicates[condition.attribute_name].end(); pit++) {

				// set parameter label to predicate label
				diagnostic_msgs::KeyValue param;
				param.key = *pit;

				// find label as it is in domain operator
				std::string conditionKey = (*cit)[index];
				index++;

				// set value
				std::vector<diagnostic_msgs::KeyValue>::iterator opit;
				for(opit = node.dispatch_msg.parameters.begin(); opit!=node.dispatch_msg.parameters.end(); opit++) {
					if(0==opit->key.compare(conditionKey)) {
						param.value = opit->value;
						ss << " " << param.value;
					}
				}
				condition.values.push_back(param);
			}

			// create new edge
			StrlEdge edge;
			edge.signal_type = CONDITION;
			edge.edge_name = ss.str();
			edge.active = false;
			edge.sinks.push_back(node.node_name);
			node.output.push_back(edge.edge_name);
			plan_edges[edge.edge_name] = edge;

			// save condition
			edge_conditions[edge.edge_name] = condition;
		}
	}

	/*------------*/
	/* Parse plan */
	/*------------*/

	/**
	 * Parse a plan written by CFF
	 */
	void CFFPlanParser::preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) {

		ROS_INFO("KCL: (CFFPlanParser) Loading plan from file: %s. Initial action ID: %zu", ((dataPath + "plan.pddl").c_str()), freeActionID);
		
		// prepare plan
		plan_nodes.clear();
		plan_edges.clear();
		std::vector<std::string> parentStack;

		// load plan file
		std::ifstream infile((dataPath + "plan.pddl").c_str());
		std::string line;
		int curr,next,nodeCount;
		bool planFound = false;
		bool planRead = false;
		
		while(!infile.eof()) {
			std::getline(infile, line);
			toLowerCase(line);

			if (line.compare("ff: found legal plan as follows") == 0) {
				planFound = true;
			} else if (!planFound) {
				//consume useless lines
			} else if (!planRead) {

				bool observeAction = false;
				bool shedAction = false;
				nodeCount = freeActionID;
				std::vector<std::string> parentEdge;

				while(!infile.eof()) {

					std::getline(infile, line);

					if(line.substr(0,10).compare("time spent")==0)
						break;

					if (line.length()<10)
						continue;

					// action name
					curr = line.find(":");
					std::string name = line.substr(curr+2).c_str();

					// deal with branches
					if("RAMINIFICATE" == name && observeAction) {

						// branch
						parentStack.push_back(parentStack.back());
						observeAction = true;

					} else if("SHED" == name.substr(0,4)) {

						// shed
						observeAction = false;
						shedAction = true;

					} else if("POP" == name.substr(0,3)) {

						// pop
						if(parentStack.size()>1) {
							std::swap(
								parentStack[parentStack.size()-1],
								parentStack[parentStack.size()-2]);
							observeAction = false;
						} else {
							ROS_INFO("KCL: (CFFPlanParser) Error parsing plan (POP)");
						}

					} else if("RAMINIFICATE" == name) {
						// skip administration actions
						observeAction = true;
					} else if("ASSUME" == name.substr(0,6)) {
						// skip administration actions
						observeAction = false;

					} else {

						StrlNode node;
						node.node_name = name;
						node.node_id = nodeCount;
						node.dispatched = false;
						node.completed = false;

						ROS_INFO("KCL: (CFFPlanParser) Created plan node: [%i] %s", node.node_id, name.c_str());

						// incoming edge(s)
						if(parentStack.size()>0) {
							node.input.push_back(parentStack.back());
							node.await_input.push_back(false);
							plan_edges[parentStack.back()].sinks.push_back(node.node_name);
							parentStack.pop_back();
						}
						if(shedAction && parentStack.size()>0) {
							node.input.push_back(parentStack.back());
							node.await_input.push_back(false);
							plan_edges[parentStack.back()].sinks.push_back(node.node_name);
							parentStack.pop_back();
							shedAction = false;
						}

						// save this parent edge
						StrlEdge edge;
						edge.signal_type = ACTION;
						std::stringstream ss;
						ss << "e" << node.node_id;
						edge.edge_name = ss.str();
						edge.sources.push_back(node.node_name);
						edge.active = false;
						plan_edges[edge.edge_name] = edge;

						node.output.push_back(edge.edge_name);
						parentStack.push_back(edge.edge_name);
						nodeCount++;

						// prepare message
						node.dispatch_msg.action_id = node.node_id;
						node.dispatch_msg.duration = 0.1;
						node.dispatch_msg.dispatch_time = 0;
						node.dispatch_msg.name = name;

						// check for parameters
						curr = line.find(":")+2;
						bool paramsExist = (line.find(" ",curr) != std::string::npos);
						if(paramsExist) {

							// name
							next = line.find(" ",curr);
							node.dispatch_msg.name = line.substr(curr,next-curr).c_str();

							// parameters
							std::vector<std::string> params;
							while(next < line.length()) {
								curr = next + 1;
								next = line.find(" ",curr);
								if(next == std::string::npos)
									next = line.length();
								diagnostic_msgs::KeyValue pair;
								pair.key = "";
								pair.value = line.substr(curr,next-curr);
								node.dispatch_msg.parameters.push_back(pair);
							}
						}
						plan_nodes[node.node_name] = node;
						observeAction = true;
					}
				}
				planRead = true;
			}
		}
		// printPlan(plan);
		// produceEsterel();
		infile.close();
	}

	/*-----------------*/
	/* Produce Esterel */
	/*-----------------*/

	/*
	 * output a plan as an Esterel controller
	 */
	bool CFFPlanParser::produceEsterel() {

		// output file
		std::string strl_file;
		ros::NodeHandle nh("~");
		nh.param("strl_file_path", strl_file, std::string("common/plan.strl"));
		
		ROS_INFO("KCL: (CFFPlanParser) Write the esterel plan: %s", strl_file.c_str());
		
		std::ofstream dest;
		dest.open(strl_file.c_str());

		// main module
		dest << "module plan:" << std::endl;

		// inputs
		dest << "input SOURCE";
		std::map<std::string,StrlNode>::iterator nit = plan_nodes.begin();
		for(; nit!=plan_nodes.end(); nit++) {
			dest << ", a" << (nit->second).node_id << "_complete";
		}
		dest << std::endl;

		// outputs
		dest << "output SINK";		
		for(nit = plan_nodes.begin(); nit!=plan_nodes.end(); nit++) {
			dest << ", a" << (nit->second).node_id << "_dispatch";
		}
		dest << std::endl;

		// internal signals
		std::map<std::string,StrlEdge>::iterator eit = plan_edges.begin();
		if(eit!=plan_edges.end()) {
			dest << "signal " << (eit->second).edge_name;
			for(; eit!=plan_edges.end(); eit++) {
				dest << ", " << (eit->second).edge_name;
			}
			dest << " in" << std::endl;
		}

		// run everything
		nit = plan_nodes.begin();
		if(nit!=plan_nodes.end()) {
			dest << "run action" << (nit->second).node_id << std::endl;
			for(; nit!=plan_nodes.end(); nit++) {
				dest << " || action" << (nit->second).node_id << std::endl;
			}
			dest << "end" << std::endl;
		}
		dest << "end module" << std::endl << std::endl;

		// action modules
		nit = plan_nodes.begin();
		for(; nit!=plan_nodes.end(); nit++) {

			dest << "module action" << (nit->second).node_id << ":" << std::endl;

			if((nit->second).input.size() > 0) {
				dest << "input ";
				for(int j=0;j<(nit->second).input.size();j++) {
					if(j>0) dest << ", ";
					dest << (nit->second).input[j];
				}
				dest << ";" << std::endl;
			}

			if((nit->second).output.size() > 0) {
				dest << "output ";
				for(int j=0;j<(nit->second).output.size();j++) {
					if(j>0) dest << ", ";
					dest << (nit->second).output[j];
				}
				dest << ";" << std::endl;
			}

			if((nit->second).input.size() > 0) {
				dest << "  await ";
				for(int j=0;j<(nit->second).input.size();j++) {
					if(j>0) dest << " or ";
					dest << (nit->second).input[j];
				}
				dest << ";" << std::endl;
			}

			dest << "  emit a" << (nit->second).node_id << "_dispatch;" << std::endl;
			dest << "  await a" << (nit->second).node_id << "_complete;" << std::endl;
			
			for(int j=0;j<(nit->second).output.size();j++) {
				dest << "emit " << (nit->second).output[j];
			}
			dest << ";" << std::endl;
			dest << "end module" << std::endl << std::endl;
		}
		dest.close();
	}
} // close namespace
