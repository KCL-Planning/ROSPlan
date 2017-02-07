#include "rosplan_planning_system/CLGPlanParser.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <stdlib.h>
#include <algorithm>
#include <ctype.h>

/* implementation of rosplan_planning_system::CLGPlanParser.h */
namespace KCL_rosplan {

	/* constructor */
	CLGPlanParser::CLGPlanParser(ros::NodeHandle &nh) : node_handle(&nh)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		parseDomain();
	}

	void CLGPlanParser::reset() {
		plan_nodes.clear();
		plan_edges.clear();
	}

	void CLGPlanParser::generateFilter(PlanningEnvironment &environment) {
		// do nothing yet
	}

	void CLGPlanParser::toLowerCase(std::string &str) {
		std::transform(str.begin(), str.end(), str.begin(), tolower);
	}

	/*--------------*/
	/* parse Domain */
	/*--------------*/

	/**
	 * This method extracts the observation expressions from the domain, and maps them to operators.
	 * This is required as VAL does not parse the operators in the CLG-style PDDL.
	 * The expressions will be added as the CONDITIONAL edge constraints.
	 */
	void CLGPlanParser::parseDomain() {
		
		ros::NodeHandle nh;

		std::string domain_path;
		nh.param("/rosplan/domain_path", domain_path, std::string("common/domain.pddl"));

		std::ifstream infile((domain_path).c_str());
		std::string line;
		std::string action_name;
		while(!infile.eof()) {

			std::getline(infile, line);
			toLowerCase(line);

			std::string::size_type n = line.find("(:action ");
			if (n != std::string::npos) {
 				n = n + 9;

				action_name = line.substr(n);
				int count = 0;
				for(std::string::iterator it = action_name.begin(); it != action_name.end(); ++it) {
				    if (*it == ' ')
						break;
					count++;
				}

				action_name = action_name.substr(0,count);
				// std::cout << action_name << std::endl;
			}

			operator_parameter_map[action_name];

			n = line.find(":parameters ");
			if (n != std::string::npos) {
 				n = n + 13;
				std::string params = line.substr(n);
				int count_start = 0;
				int count_length = 0;
				bool finished = false;
				int state = 0;
				while(!finished && !infile.eof()) {
					for(std::string::iterator it = params.begin(); it != params.end(); ++it) {

						switch (state) {
						case 0: // reading parameter
							if (*it == '-' || *it == ' ') {
								// std::cout << params.substr(count_start,count_length) << std::endl;
								operator_parameter_map[action_name].push_back(params.substr(count_start,count_length));
								state++;
							}
							break;
						case 1: // skipping " - "
							if (*it != '-' && *it != ' ') state++;
							break;
						case 2: // skipping type
							if (*it == ' ') state++;
							break;
						case 3: // skipping spaces
							if (*it != ' ') {
								count_start = count_start + count_length;
								count_length = 0;
								state = 0;
							}
							break;
						}

						count_length++;

						if (*it == ')') {
							finished = true;
						}
					}

					if(!finished) {
						std::getline(infile, params);
						toLowerCase(params);
					}
				}
			}

			n = line.find(":observe ");
			std::stringstream obs;
			if (n != std::string::npos) {
 				n = n + 9;
				std::string observation = line.substr(n);
				int count = 0;
				int bracket_depth = 0;
				int last_close = 0;
				bool started = false;
				while((!started || bracket_depth > 0) && !infile.eof()) {
					for(std::string::iterator it = observation.begin(); it != observation.end(); ++it) {
					    if (*it == '(') {
							bracket_depth++;
							started = true;
						} else if (*it == ')') {
							bracket_depth--;
							if(bracket_depth>=0)
								last_close = count;
						}
						count++;
					}
					
					if(bracket_depth>=0) {
						obs << observation;
						std::getline(infile, observation);
						toLowerCase(observation);
					} else {
						obs << observation.substr(0,last_close);
					}
				}

				operator_observation_map[action_name] = obs.str();
			}

		}
	}

	/*------------*/
	/* parse PDDL */
	/*------------*/

	/**
	 * parse a PDDL condition
	 */
	void CLGPlanParser::preparePDDLConditions(StrlEdge* last_edge, StrlEdge* other_edge, StrlNode* node) {

		std::string operator_name = node->node_name.substr(0, node->node_name.find("_"));

		// gather parameters
		std::vector<std::string> parameters;
		if(node->node_name.find("_") != std::string::npos) {

			std::string param = node->node_name.substr(1+node->node_name.find("_"));
			while(param.find("_") != std::string::npos) {
				parameters.push_back(param.substr(0,param.find("_")));
				param = param.substr(1+param.find("_"));
			}
			parameters.push_back(param);

		}

		// generate grounded observation
		std::stringstream grounded_observation;
		char* str = strdup(operator_observation_map[operator_name].c_str());
		const char s[2] = " ";
		char* token;

		// get the first token
		token = std::strtok(str, s);

		// walk through other tokens
		while( token != NULL ) {
			bool found = false;
			for(int i=0; i<parameters.size(); i++) {
				std::string tok = std::string(token);
				if(tok.find(")")!=std::string::npos)
					tok = tok.substr(0,tok.find(")"));
				if(tok == operator_parameter_map[operator_name][i]) {
					grounded_observation << parameters[i] << " ";
					found = true;
					break;
				}
			}
			if(!found) {
				grounded_observation << token << " ";
			} else {
				std::string tok = std::string(token);
				if(tok.find(")")!=std::string::npos)
					grounded_observation << tok.substr(tok.find(")"));
			}
			token = std::strtok(NULL, s);
		}
			
		// update edges
		last_edge->signal_type = CONDITION;
		last_edge->edge_name = grounded_observation.str();

		std::stringstream negedge;
		negedge << "(not " << grounded_observation.str() << ")";
		other_edge->signal_type = CONDITION;
		other_edge->edge_name = negedge.str();
	}

	/*------------*/
	/* Parse plan */
	/*------------*/
	
	void CLGPlanParser::createNodeAndEdge(const std::string& action_name, int node_id, PlanningEnvironment &environment, StrlNode& node, StrlEdge& edge)
	{
		node.node_name = action_name;
		node.node_id = node_id;
		node.dispatched = false;
		node.completed = false;

		// save this parent edge
		edge.signal_type = ACTION;
		std::stringstream ss;
		ss << "e_" << node.node_id;
		edge.edge_name = ss.str();
		edge.sources.push_back(&node);
		edge.active = false;
		plan_edges.push_back(&edge);
		
		// prepare message
		std::string operator_name = action_name.substr(0, action_name.find("_"));
		node.output.push_back(&edge);
		node.dispatch_msg.action_id = node.node_id;
		node.dispatch_msg.duration = 0.1;
		node.dispatch_msg.dispatch_time = 0;
		node.dispatch_msg.name = operator_name;

		plan_nodes.push_back(&node);
	}

	/**
	 * Parse a plan written by CLG
	 */
	void CLGPlanParser::preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) {

		ROS_INFO("KCL: (CLGPlanParser) Loading plan from file: %s. Initial action ID: %zu", ((dataPath + "plan.pddl").c_str()), freeActionID);
		
		// prepare plan
		plan_nodes.clear();
		plan_edges.clear();
		
		// stack of branches
		std::vector<StrlEdge*> branches;

		// load plan file
		std::ifstream infile((dataPath + "plan.pddl").c_str());
		std::string line;
		int curr,next,nodeCount;
		bool planFound = false;
		bool planRead = false;
		
		nodeCount = freeActionID;
		StrlEdge* last_edge = NULL;

		while(!infile.eof()) {

			std::getline(infile, line);
			toLowerCase(line);

			if(line.substr(0,41).compare("::::::::::::::::::::::::current action:::")==0) {
				
				// parse action
				std::string action_name = line.substr(41);

				StrlNode* node = new StrlNode();
				StrlEdge* edge = new StrlEdge();
				createNodeAndEdge(action_name, nodeCount, environment, *node, *edge);
				++nodeCount;

				if (last_edge != NULL) {
					node->input.push_back(last_edge);
					last_edge->sinks.push_back(node);
				}
				last_edge = edge;

			} else if(line.compare("branching....")==0) {

				if (last_edge == NULL) {
					std::cerr << "Branch without any preceding actions!" << std::endl;
				}

				// branch point
				StrlEdge* edge = new StrlEdge();

				edge->signal_type = ACTION;
				edge->edge_name = last_edge->edge_name;
				edge->active = false;

				std::vector<StrlNode*>::iterator sit = last_edge->sources.begin();
				for(; sit!=last_edge->sources.end(); sit++)
					edge->sources.push_back(*sit);

				plan_edges.push_back(edge);
				branches.push_back(edge);

				// add conditions on the branches
				preparePDDLConditions(last_edge, edge, *last_edge->sources.begin());


			} else if(line.substr(0,24).compare("goal reached in a branch")==0) {

				// return to last branch
				if(branches.size() > 0) {
					last_edge = branches[branches.size()-1];
					branches.pop_back();
				}

			} else {

				// consume useless lines

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
	bool CLGPlanParser::produceEsterel() {

		// output file
		std::string strl_file;
		ros::NodeHandle nh("~");
		nh.param("/rosplan/strl_file_path", strl_file, std::string("common/plan.strl"));
		
		ROS_INFO("KCL: (CLGPlanParser) Write the esterel plan: %s", strl_file.c_str());
		
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
