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

	/*---------------------*/
	/* string manipulation */
	/*---------------------*/

	void CFFPlanParser::toLowerCase(std::string &str) {
		std::transform(str.begin(), str.end(), str.begin(), tolower);
	}

	unsigned int CFFPlanParser::split(const std::string &txt, std::vector<std::string> &strs, char ch) {
		unsigned int pos = txt.find( ch );
		unsigned int initialPos = 0;
		strs.clear();
		// Decompose statement
		while( pos != std::string::npos && pos < txt.length()) {
			if(txt.substr( initialPos, pos - initialPos + 1 ) !=" ") {
				std::string s = txt.substr( initialPos, pos - initialPos + 1 );
				s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
			    strs.push_back(s);
			}
		    initialPos = pos + 1;
		    pos = txt.find( ch, initialPos );
		}
		// Add the last one
		strs.push_back( txt.substr( initialPos, txt.size() - initialPos ) );
		return strs.size();
	}

	/*------------*/
	/* parse PDDL */
	/*------------*/

	/**
	 * parse a PDDL condition
	 */
	void CFFPlanParser::preparePDDLConditions(StrlNode &node, PlanningEnvironment &environment) {

	}

	/*------------*/
	/* Parse plan */
	/*------------*/
	
	void CFFPlanParser::createNode(std::vector<std::string> &s, const std::string& action_name, int node_id, PlanningEnvironment &environment, StrlNode& node) {

		// concatenate parameters
		std::stringstream ss;
		ss << action_name << " ";
		int i=3;
		while (s[i]!="---") {
			ss << s[i] << " ";
			i++;
		}

		node.node_name = ss.str();
		node.node_id = node_id;
		node.dispatched = false;
		node.completed = false;

		// prepare message
		node.dispatch_msg.action_id = node.node_id;
		node.dispatch_msg.duration = 0.1;
		node.dispatch_msg.dispatch_time = 0;
		node.dispatch_msg.name = action_name;

		// TODO preparePDDLConditions

		plan_nodes.push_back(&node);
	}

	/**
	 * Parse a plan written by CFF
	 */
	void CFFPlanParser::preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) {

		ROS_INFO("KCL: (CFFPlanParser) Loading plan from file: %s. Initial action ID: %zu", ((dataPath + "plan.pddl").c_str()), freeActionID);
		
		// prepare plan
		reset();
		
		// stack of branches
		std::vector<StrlEdge*> branches;

		// load plan file
		std::ifstream infile((dataPath + "plan.pddl").c_str());
		std::string line;
		std::vector<std::string> s;
		int curr,next,nodeCount;
		bool planFound = false;
		bool planRead = false;
		
		nodeCount = freeActionID;
		StrlEdge* last_edge = NULL;

		while(!infile.eof()) {

			std::getline(infile, line);
			toLowerCase(line);

			// loop until plan is printed
			if(line.compare("ff: found plan as follows")==0) {
				planFound = true;
			}
			if(!planFound) continue;

			// skip dividers				
			if(line.compare("-------------------------------------------------")==0) {
				continue;
			}

			// parse action
			if(line.find("---", 0) != std::string::npos) {

				// actions look like this:
				// "16||1", "---", "action_name", "param1", ..., "---", "son:", "17||-1"
				// "11||1", "---", "action_name", "param1", ..., "---", "trueson:", "12||1", "---", "falseson:", "12||2"

				split(line, s, ' ');
				std::string action_name = s[2];
				std::string action_id = s[0];
				if(cff_node_map.find(action_id)==cff_node_map.end()) {
					cff_node_map[action_id] = new StrlNode();
				}


				StrlNode* node = cff_node_map[action_id];
				createNode(s, action_name, nodeCount, environment, *node);
				++nodeCount;


				// complete incoming edges
				if(incoming_edge_map.find(action_id)!=incoming_edge_map.end()) {
					for(int i=0;i<incoming_edge_map[action_id].size();i++) {
						node->input.push_back(incoming_edge_map[action_id][i]);
						incoming_edge_map[action_id][i]->sinks.push_back(node);
					}
				}

				// prepare outgoing edges
				if(s[s.size()-2].substr(0,4) == "son:") {

					// single child
					StrlEdge* edge = new StrlEdge();
					
					// save this parent edge
					edge->signal_type = ACTION;
					std::stringstream ss;
					ss << "e_" << node->node_id;
					edge->edge_name = ss.str();
					edge->sources.push_back(node);
					edge->active = false;
					plan_edges.push_back(edge);
					node->output.push_back(edge);

					// prepare for child
					incoming_edge_map[s[s.size()-1]];
					incoming_edge_map[s[s.size()-1]].push_back(edge);

				} else if(s[s.size()-2].substr(0,9) == "falseson:") {

					for(int i=0;i<2;i++) {

						// two children
						StrlEdge* edge = new StrlEdge();
					
						// save this parent edge
						edge->signal_type = ACTION;
						std::stringstream ss;
						ss << "e_" << node->node_id << "_" << i;
						edge->edge_name = ss.str();
						edge->sources.push_back(node);
						edge->active = false;
						plan_edges.push_back(edge);
						node->output.push_back(edge);

						// prepare for child
						incoming_edge_map[s[s.size()-1-i*3]];
						incoming_edge_map[s[s.size()-1-i*3]].push_back(edge);
					}
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
	bool CFFPlanParser::produceEsterel() {
		// nothing right now...
	}
} // close namespace
