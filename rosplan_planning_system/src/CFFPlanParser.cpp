#include "rosplan_planning_system/CFFPlanParser.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>
#include <algorithm>
#include <ctype.h>

/* implementation of rosplan_planning_system::CFFPlanParser.h */
namespace KCL_rosplan {

	/* constructor */
	CFFPlanParser::CFFPlanParser(ros::NodeHandle &nh) : node_handle(&nh)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	}

	void CFFPlanParser::reset() {
		plan_nodes.clear();
		plan_edges.clear();
	}

	void CFFPlanParser::generateFilter(PlanningEnvironment &environment) {
		// do nothing yet
	}

	void CFFPlanParser::toLowerCase(std::string &str) {
		std::transform(str.begin(), str.end(), str.begin(), tolower);
	}

	/*------------*/
	/* parse PDDL */
	/*------------*/

	/**
	 * parse a PDDL condition
	 */
	void CFFPlanParser::preparePDDLConditions(StrlNode &node, PlanningEnvironment &environment) {

		std::string normalised_action_name = node.dispatch_msg.name;
		std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
		
		// find action conditions
		std::map<std::string, std::vector<std::vector<std::string> > >::iterator oit;
		oit = environment.domain_operator_precondition_map.find(normalised_action_name);
		if(oit==environment.domain_operator_precondition_map.end()) {
			std::cerr << "action precondition map entry not found:" << node.dispatch_msg.name << std::endl;
			for (std::map<std::string, std::vector<std::vector<std::string> > >::const_iterator ci = environment.domain_operator_precondition_map.begin(); ci != environment.domain_operator_precondition_map.end(); ++ci)
			{
				std::cerr << (*ci).first;
				const std::vector<std::vector<std::string> >& mapping = (*ci).second;
				for (std::vector<std::vector<std::string> >::const_iterator ci = mapping.begin(); ci != mapping.end(); ++ci)
				{
					const std::vector<std::string>& list = *ci;
					std::cerr << "{";
					for (std::vector<std::string>::const_iterator ci = list.begin(); ci != list.end(); ++ci)
					{
						std::cerr << *ci << ", ";
					}
					std::cerr << "}" << std::endl;
				}
			}
			exit(1);
			return;
		}
		
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> processed_preconditions;

		// iterate through conditions
		for(std::vector<std::vector<std::string> >::iterator cit = oit->second.begin(); cit!=oit->second.end(); cit++) {
			
			rosplan_knowledge_msgs::KnowledgeItem condition;
			condition.is_negative = false;
			
			// set fact or function
			std::map<std::string,std::vector<std::string> >::iterator dit = environment.domain_predicates.find((*cit)[0]);
			if(dit!=environment.domain_predicates.end()) condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;

			dit = environment.domain_functions.find((*cit)[0]);
			if(dit!=environment.domain_functions.end()) condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;

			// create edge name
			condition.attribute_name = (*cit)[0];
			
			// Preconditions that start with an 'r' are resolved atoms that are only necessary for generating the plan.
			if (condition.attribute_name[0] == 'r' && "resolve-axioms" != condition.attribute_name && "robot_at" != condition.attribute_name)
			{
				condition.attribute_name = condition.attribute_name.substr(1);
			}
			
			std::stringstream ss;
			ss << condition.attribute_name;

			// populate parameters
			int index = 1;
			
			for(std::vector<std::string>::iterator pit = environment.domain_predicates[condition.attribute_name].begin(); pit!=environment.domain_predicates[condition.attribute_name].end(); pit++) {

				// set parameter label to predicate label
				diagnostic_msgs::KeyValue param;
				param.key = *pit;

				// find label as it is in domain operator
				std::string conditionKey = (*cit)[index];
				std::transform(conditionKey.begin(), conditionKey.end(), conditionKey.begin(), tolower);
				index++;

				// set value
				std::vector<diagnostic_msgs::KeyValue>::iterator opit;
				for(opit = node.dispatch_msg.parameters.begin(); opit!=node.dispatch_msg.parameters.end(); opit++) {
					
					std::string parameter = opit->key;
					std::transform(parameter.begin(), parameter.end(), parameter.begin(), tolower);
					
					if(0==parameter.compare(conditionKey)) {
						param.value = opit->value;
						ss << " " << param.value;
					}
				}
				condition.values.push_back(param);
			}
			
			// Make sure we haven't already processed this precondition.
			bool already_processed = true;
			for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = processed_preconditions.begin(); ci != processed_preconditions.end(); ++ci)
			{
				const rosplan_knowledge_msgs::KnowledgeItem& processed_precondition = *ci;
				if (condition.attribute_name != processed_precondition.attribute_name)
				{
					already_processed = false;
					break;
				}
				
				for (unsigned int i = 0; i < processed_precondition.values.size(); ++i)
				{
					if (processed_precondition.values[i].value != condition.values[i].value)
					{
						already_processed = false;
						break;
					}
				}
			}
			
			if (already_processed) continue;
			
			// create new edge
			StrlEdge* edge = new StrlEdge();
			edge->signal_type = CONDITION;
			edge->edge_name = ss.str();
			edge->active = false;
			edge->sinks.push_back(&node);
			edge->external_conditions.push_back(condition);
			
			node.input.push_back(edge);
			plan_edges.push_back(edge);
		}
	}

	/*------------*/
	/* Parse plan */
	/*------------*/
	
	void CFFPlanParser::createNodeAndEdge(const std::string& action_name, int action_number, int node_id, PlanningEnvironment &environment, StrlNode& node, StrlEdge& edge)
	{
		node.node_name = action_name;
		node.node_id = node_id;
		node.dispatched = false;
		node.completed = false;

		// save this parent edge
		edge.signal_type = ACTION;
		std::stringstream ss;
		ss << "e" << node.node_id;
		edge.edge_name = ss.str();
		edge.sources.push_back(&node);
		edge.active = false;
		plan_edges.push_back(&edge);
		
		// prepare message
		node.output.push_back(&edge);
		node.dispatch_msg.action_id = node.node_id;
		node.dispatch_msg.duration = 0.1;
		node.dispatch_msg.dispatch_time = 0;
		node.dispatch_msg.name = action_name;
		
		std::string operator_name = action_name.substr(0, action_name.find(" "));
		
		// check for parameters
		int curr = operator_name.length();
		int next = 0;
		bool paramsExist = (action_name.find(" ",curr) != std::string::npos);
		if(paramsExist) {

			// name
			next = action_name.find(" ",curr);
			node.dispatch_msg.name = operator_name;
			int parameter_index = 0;
			
			// parameters
			std::vector<std::string> params;
			while(next < action_name.length()) {
				curr = next + 1;
				next = action_name.find(" ",curr);
				if(next == std::string::npos)
					next = action_name.length();
				
				diagnostic_msgs::KeyValue pair;
				pair.key = environment.domain_operators[operator_name][parameter_index];
				pair.value = action_name.substr(curr,next-curr);
				node.dispatch_msg.parameters.push_back(pair);
				++parameter_index;
			}
		}
		preparePDDLConditions(node, environment);
		plan_nodes.push_back(&node);
		jump_map.insert(std::pair<int, StrlNode*>(action_number,&node));
	}

	/**
	 * Parse a plan written by CFF
	 */
	void CFFPlanParser::preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) {

		ROS_INFO("KCL: (CFFPlanParser) Loading plan from file: %s. Initial action ID: %zu", ((dataPath + "plan.pddl").c_str()), freeActionID);
		
		// prepare plan
		plan_nodes.clear();
		plan_edges.clear();
		
		// Manage shed / assume knowledge bases.
		std::vector<std::vector<StrlEdge*>* > leafs;
		
		// Keep a stack of the observation actions that still have to be resolved.
		std::vector<StrlEdge*> conditional_branch_stack;
		
		// Keep a stack of the negative / possitive observations made.
		std::vector<StrlEdge*> active_branch_edge;
		
		// The last edge that will lead to the next action.
		StrlEdge* last_edge = NULL;
		
		bool observation_is_active = false;
		bool last_node_is_jump = false;

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
				
				nodeCount = freeActionID;
				std::vector<std::string> parentEdge;

				while(!infile.eof()) {

					std::getline(infile, line);
					toLowerCase(line);
					
					if(line.substr(0,10).compare("time spent")==0)
						break;

					if (line.length()<10)
						continue;

					// action name
					curr = line.find(":");
					std::string name = line.substr(curr+2);

					// action number
					int action_number = atoi(line.substr(5).c_str());

					// deal with branches
					if("ramificate" == name) {
						continue;
					} else if("shed" == name.substr(0,4)) {

						// Create a node for this action.
						StrlNode* node = new StrlNode();
						StrlEdge* edge = new StrlEdge();
						
						createNodeAndEdge(name, action_number, nodeCount, environment, *node, *edge);
						++nodeCount;
						
						// Shedding a knowledge base indicates that there 
						// are no more states on the stack, so we can make
						// this action follow all of the leafs.
						std::vector<StrlEdge*>* current_leafs = leafs[leafs.size() - 1];
						if (!last_node_is_jump)
						{
							current_leafs->push_back(last_edge);
						}
						leafs.pop_back();
						
						// Make the shed action the next action for all leafs.
						for (std::vector<StrlEdge*>::const_iterator ci = current_leafs->begin(); ci != current_leafs->end(); ++ci)
						{
							StrlEdge* leaf_edge = *ci;
							
							node->input.push_back(leaf_edge);
							leaf_edge->sinks.push_back(node);
						}
						delete current_leafs;
						
						last_edge = edge;
						observation_is_active = false;

					} else if("pop" == name.substr(0,3)) {
						
						std::vector<StrlEdge*>* current_leafs = leafs[leafs.size() - 1];
						current_leafs->push_back(last_edge);
						
						// When a pop action is executed we go back to the last node where we 
						// branched due to an observation that has been made.
						last_edge = conditional_branch_stack[conditional_branch_stack.size() - 1];
						conditional_branch_stack.pop_back();
						observation_is_active = true;
						
					} else if("assume" == name.substr(0,6)) {
						// When a knowledge base is assumed, we need to prepare the dispatches for the 
						// 'shed' action. When this action occurs we need to link all the 'leafs' of 
						// all branches that were formed whilst this knowledge base was active to the 
						// shed action.
						std::vector<StrlEdge*>* new_knowledge_base = new std::vector<StrlEdge*>();
						leafs.push_back(new_knowledge_base);
						observation_is_active = false;

					} else if("jump" == name.substr(0,4)) {

						StrlNode* node = new StrlNode();
						StrlEdge* edge = new StrlEdge();
						createNodeAndEdge(name, action_number,  nodeCount, environment, *node, *edge);
						++nodeCount;
						
						if (last_edge != NULL)
						{
							node->input.push_back(last_edge);
							last_edge->sinks.push_back(node);
						}
						last_edge = edge;

						// Check if this action is dependend on an observation outcome.
						if (observation_is_active)
						{
							StrlEdge* conditional_edge = active_branch_edge[active_branch_edge.size() - 1];
							node->input.push_back(conditional_edge);
							conditional_edge->sinks.push_back(node);
							observation_is_active = false;
							active_branch_edge.pop_back();
						}

						// connect output edge to jump location
						int jumpDest = atoi(name.substr(5).c_str());
						std::map<int,StrlNode*>::iterator it = jump_map.find(jumpDest);
						
						if (it != jump_map.end() && last_edge != NULL) {
							it->second->input.push_back(last_edge);
							last_edge->sinks.push_back(it->second);
						} else {
							ROS_INFO("KCL: (CFFPlanParser) Could not find jump destination: %i", jumpDest);
						}

						if (active_branch_edge.size() > 0)
						{
							// IF WE ARE IN A BRANCH we go back to the last node where we 
							// branched due to an observation that has been made.
							last_edge = conditional_branch_stack.back();
							conditional_branch_stack.pop_back();
							observation_is_active = true;
						}
						last_node_is_jump = true;

					} else {

						last_node_is_jump = false;

						StrlNode* node = new StrlNode();
						StrlEdge* edge = new StrlEdge();
						createNodeAndEdge(name, action_number,  nodeCount, environment, *node, *edge);
						++nodeCount;
						
						if (last_edge != NULL)
						{
							node->input.push_back(last_edge);
							last_edge->sinks.push_back(node);
						}
						last_edge = edge;
						
						// Check if this action is dependend on an observation outcome.
						if (observation_is_active)
						{
							StrlEdge* conditional_edge = active_branch_edge[active_branch_edge.size() - 1];
							node->input.push_back(conditional_edge);
							conditional_edge->sinks.push_back(node);
							observation_is_active = false;
							active_branch_edge.pop_back();
						}
						
						// Check if this is an observation action. If so then we prepare the negative and 
						// possitive branches.
						if ("observe-" == name.substr(0, 8)) {
							std::string observation_fact = name.substr(8);
							
							// Create the possitive first.
							std::stringstream ss;
							ss << name << "_TRUE";
							StrlEdge* possitive_edge = new StrlEdge();
							possitive_edge->signal_type = CONDITION;
							possitive_edge->edge_name = ss.str();
							possitive_edge->active = false;
							
							//plan_edges.push_back(edge);
							
							// Make the observation outcome a condition.
							rosplan_knowledge_msgs::KnowledgeItem condition;
							condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
							condition.is_negative = false;

							std::vector<std::string> tokens;
							size_t counter = 0;
							size_t new_counter = 0;
							while ((new_counter = observation_fact.find(" ", counter)) != string::npos)
							{
								tokens.push_back(observation_fact.substr(counter, new_counter - counter));
								counter = new_counter + 1;
							}
							tokens.push_back(observation_fact.substr(counter));
							
							// create edge name
							condition.attribute_name = tokens[0];
							
							ss.str(std::string());
							ss << condition.attribute_name;

							// Populate parameters
							int index = 1;
							std::vector<std::string>::iterator pit = environment.domain_predicates[condition.attribute_name].begin();
							for(; pit!=environment.domain_predicates[condition.attribute_name].end(); pit++) {
								// set parameter label to predicate label
								diagnostic_msgs::KeyValue param;
								param.key = *pit;
								param.value = tokens[index];
								condition.values.push_back(param);
								++index;
							}
							
							// Remove the last parameter (state).
							condition.values.erase(condition.values.begin() + condition.values.size() - 1);
							
							possitive_edge->external_conditions.push_back(condition);
							
							// Next create the negative branch.
							ss.str(std::string());
							ss << name << "_FALSE";
							StrlEdge* negative_edge = new StrlEdge(*possitive_edge);
							negative_edge->edge_name = ss.str();
							negative_edge->external_conditions.clear();
							condition.is_negative = true;
							negative_edge->external_conditions.push_back(condition);
							active_branch_edge.push_back(negative_edge);
							active_branch_edge.push_back(possitive_edge);
							
							plan_edges.push_back(negative_edge);
							plan_edges.push_back(possitive_edge);
							
							// Flag that the next action is conditional on an observation.
							observation_is_active = true;
							
							conditional_branch_stack.push_back(edge);
						}
					}
				}
				planRead = true;
			}
		}
		// printPlan(plan);
		produceEsterel();
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
		nh.param("/rosplan/strl_file_path", strl_file, std::string("common/plan.strl"));
		
		ROS_INFO("KCL: (CFFPlanParser) Write the esterel plan: %s", strl_file.c_str());
		
		std::ofstream dest;
		dest.open(strl_file.c_str());

		// main module
		dest << "module plan:" << std::endl;

		// inputs
		dest << "input SOURCE";
		std::vector<StrlNode*>::iterator nit = plan_nodes.begin();
		for(; nit!=plan_nodes.end(); nit++) {
			dest << ", a" << (*nit)->node_id << "_complete";
		}
		dest << std::endl;

		// outputs
		dest << "output SINK";		
		for(nit = plan_nodes.begin(); nit!=plan_nodes.end(); nit++) {
			dest << ", a" << (*nit)->node_id << "_dispatch";
		}
		dest << std::endl;

		// internal signals
		std::vector<StrlEdge*>::iterator eit = plan_edges.begin();
		if(eit!=plan_edges.end()) {
			dest << "signal " << (*eit)->edge_name;
			for(; eit!=plan_edges.end(); eit++) {
				dest << ", " << (*eit)->edge_name;
			}
			dest << " in" << std::endl;
		}

		// run everything
		nit = plan_nodes.begin();
		if(nit!=plan_nodes.end()) {
			dest << "run action" << (*nit)->node_id << std::endl;
			for(; nit!=plan_nodes.end(); nit++) {
				dest << " || action" << (*nit)->node_id << std::endl;
			}
			dest << "end" << std::endl;
		}
		dest << "end module" << std::endl << std::endl;

		// action modules
		nit = plan_nodes.begin();
		for(; nit!=plan_nodes.end(); nit++) {

			dest << "module action" << (*nit)->node_id << ":" << std::endl;

			if((*nit)->input.size() > 0) {
				dest << "input ";
				for(int j=0;j<(*nit)->input.size();j++) {
					if(j>0) dest << ", ";
					dest << (*nit)->input[j]->edge_name;
				}
				dest << ";" << std::endl;
			}

			if((*nit)->output.size() > 0) {
				dest << "output ";
				for(int j=0;j<(*nit)->output.size();j++) {
					if(j>0) dest << ", ";
					dest << (*nit)->output[j]->edge_name;
				}
				dest << ";" << std::endl;
			}

			if((*nit)->input.size() > 0) {
				dest << "  await ";
				for(int j=0;j<(*nit)->input.size();j++) {
					if(j>0) dest << " or ";
					dest << (*nit)->input[j]->edge_name;
				}
				dest << ";" << std::endl;
			}

			dest << "  emit a" << (*nit)->node_id << "_dispatch;" << std::endl;
			dest << "  await a" << (*nit)->node_id << "_complete;" << std::endl;
			
			for(int j=0;j<(*nit)->output.size();j++) {
				dest << "emit " << (*nit)->output[j]->edge_name;
			}
			dest << ";" << std::endl;
			dest << "end module" << std::endl << std::endl;
		}
		dest.close();
	}
} // close namespace
