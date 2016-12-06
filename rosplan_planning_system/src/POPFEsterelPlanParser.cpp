#include "rosplan_planning_system/POPFEsterelPlanParser.h"

/* implementation of rosplan_planning_system::POPFEsterelPlanParser.h */
namespace KCL_rosplan {

	/* constructor */
	POPFEsterelPlanParser::POPFEsterelPlanParser(ros::NodeHandle &nh) : node_handle(&nh)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	}

	void POPFEsterelPlanParser::reset() {
		plan_nodes.clear();
		plan_edges.clear();
	}

	void POPFEsterelPlanParser::generateFilter(PlanningEnvironment &environment) {
		// do nothing yet
	}

	void POPFEsterelPlanParser::toLowerCase(std::string &str) {
		std::transform(str.begin(), str.end(), str.begin(), tolower);
	}

	/*------------*/
	/* parse PDDL */
	/*------------*/

	/**
	 * parse a PDDL condition
	 */
	void POPFEsterelPlanParser::preparePDDLConditions(StrlNode &node, PlanningEnvironment &environment) {

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
	
	void POPFEsterelPlanParser::createNodeAndEdge(const std::string& action_name, double dispatchTime, double duration, int node_id, PlanningEnvironment &environment, StrlNode& node, StrlEdge& edge) {

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
		node.dispatch_msg.duration = duration;
		node.dispatch_msg.dispatch_time = dispatchTime;
		node.dispatch_msg.name = action_name;

		// check for parameters
		int curr = 0;
		int next = 0;	
		bool paramsExist = (action_name.find(" ",curr) != std::string::npos);	
		if(paramsExist) {

			// name
			next = action_name.find(" ",curr);
			node.dispatch_msg.name = action_name.substr(curr,next-curr).c_str();

			// parameters
			int parameter_index = 0;
			std::vector<std::string> params;
			while(next < action_name.length()) {

				curr = next + 1;
				next = action_name.find(" ",curr);
				if(next == std::string::npos)
					next = action_name.length();
				
				diagnostic_msgs::KeyValue pair;
				pair.key = environment.domain_operators[node.dispatch_msg.name][parameter_index];
				pair.value = action_name.substr(curr,next-curr);
				node.dispatch_msg.parameters.push_back(pair);
				++parameter_index;
			}
		}
		preparePDDLConditions(node, environment);
		plan_nodes.push_back(&node);
		action_list.push_back(node.dispatch_msg);
	}

	/**
	 * Parse a plan written by POPF
	 */
	void POPFEsterelPlanParser::preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) {

		ROS_INFO("KCL: (POPFEsterelPlanParser) Loading plan from file: %s. Initial action ID: %zu", ((dataPath + "plan.pddl").c_str()), freeActionID);

		// load plan file
		StrlEdge* last_edge = NULL;
		std::ifstream infile((dataPath + "plan.pddl").c_str());
		int curr,next,nodeCount;
		std::string line;
		double expectedPlanDuration = 0;
		bool planFound = false;
		bool planRead = false;

		while(!infile.eof()) {

			std::getline(infile, line);
			toLowerCase(line);
			
			if (line.substr(0,6).compare("; plan") == 0) {

				// prepare plan
				plan_nodes.clear();
				plan_edges.clear();
				action_list.clear();
				expectedPlanDuration = atof(line.substr(25).c_str());
				planFound = true;
				planRead = false;

				// The last edge that will lead to the next action.
				last_edge = NULL;

			} else if (line.substr(0,6).compare("; time")!=0) {
				//consume useless lines

			} else if (!planRead) {

				nodeCount = freeActionID;
				std::vector<std::string> parentEdge;

				while(!infile.eof() && line.compare("")!=0) {

					std::getline(infile, line);
					toLowerCase(line);

					// 0.000: (goto_waypoint kenny wp0 wp1)  [10.000]					
					if (line.length()<2)
						break;

					// dispatchTime
					curr = line.find(":");
					double dispatchTime = (double)atof(line.substr(0,curr).c_str());
					curr += 3;

					// action
					next=line.find(")",curr);
					std::string name = line.substr(curr,next-curr).c_str();

					// duration
					curr=line.find("[",curr)+1;
					next=line.find("]",curr);
					double duration = (double)atof(line.substr(curr,next-curr).c_str());
					{
						StrlNode* node = new StrlNode();
						StrlEdge* edge = new StrlEdge();
						createNodeAndEdge(name, dispatchTime, duration, nodeCount, environment, *node, *edge);
						++nodeCount;

						if (last_edge != NULL)
						{
							node->input.push_back(last_edge);
							last_edge->sinks.push_back(node);
						}
						last_edge = edge;
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
	bool POPFEsterelPlanParser::produceEsterel() {

		// output file
		std::string strl_file;
		ros::NodeHandle nh("~");
		nh.param("strl_file_path", strl_file, std::string("common/plan.strl"));
		
		ROS_INFO("KCL: (POPFEsterelPlanParser) Write the esterel plan: %s", strl_file.c_str());
		
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
			dest << "run action" << (*(nit++))->node_id << std::endl;
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
					if(j>0) dest << " and ";
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
