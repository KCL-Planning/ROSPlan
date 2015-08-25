#include "rosplan_planning_system/PlanParser.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

namespace KCL_rosplan {

	void PlanParser::reset() {
		filter_objects.clear();
		filter_attributes.clear();
		knowledge_filter.clear();
		action_list.clear();
	}

	/*----------------------*/
	/* Post processing plan */
	/*----------------------*/

	/**
	 * parses the output of popf, generating a list of action messages.
	 */
	void PlanParser::preparePlan(std::string &dataPath, PlanningEnvironment &environment, size_t freeActionID) {

		// trim the end of any existing plan
		while(action_list.size() > freeActionID)
			action_list.pop_back();

		// popf output
		std::ifstream planfile;
		planfile.open((dataPath + "plan.pddl").c_str());
		
		int curr, next; 
		std::string line;
		std::vector<rosplan_dispatch_msgs::ActionDispatch> potentialPlan;
		double planDuration;
		double expectedPlanDuration = 0;

		while(!planfile.eof()) {

			getline(planfile, line);

			if (line.substr(0,6).compare("; Plan") == 0) {
				expectedPlanDuration = atof(line.substr(25).c_str());
			} else if (line.substr(0,6).compare("; Time")!=0) {
				//consume useless lines
			} else {

				potentialPlan.clear();
				size_t planFreeActionID = freeActionID;
				planDuration = 0;

				while(!planfile.eof() && line.compare("")!=0) {

					getline(planfile, line);
					if (line.length()<2)
						break;

					rosplan_dispatch_msgs::ActionDispatch msg;

					// action ID
					msg.action_id = planFreeActionID;
					planFreeActionID++;

					// dispatchTime
					curr=line.find(":");
					double dispatchTime = (double)atof(line.substr(0,curr).c_str());
					msg.dispatch_time = dispatchTime;

					// check for parameters
					curr=line.find("(")+1;
					bool paramsExist = (line.find(" ",curr) < line.find(")",curr));

					if(paramsExist) {

						// name
						next=line.find(" ",curr);
						std::string name = line.substr(curr,next-curr).c_str();
						msg.name = name;

						// parameters
						std::vector<std::string> params;
						curr=next+1;
						next=line.find(")",curr);
						int at = curr;
						while(at < next) {
							int cc = line.find(" ",curr);
							int cc1 = line.find(")",curr);
							curr = cc<cc1?cc:cc1;
							std::string param = environment.name_map[line.substr(at,curr-at)];
							params.push_back(param);
							++curr;
							at = curr;
						}
						processPDDLParameters(msg, params, environment);


					} else {

						// name
						next=line.find(")",curr);
						std::string name = line.substr(curr,next-curr).c_str();
						msg.name = name;

					}

					// duration
					curr=line.find("[",curr)+1;
					next=line.find("]",curr);
					msg.duration = (double)atof(line.substr(curr,next-curr).c_str());

					potentialPlan.push_back(msg);

					// update plan duration
					curr=line.find(":");
					planDuration = msg.duration + atof(line.substr(0,curr).c_str());
				}

				if(planDuration - expectedPlanDuration < 0.01)  {

					// trim any previously read plan
					while(action_list.size() > freeActionID) {
						action_list.pop_back();
					}

					// save better optimised plan
					for(size_t i=0;i<potentialPlan.size();i++) {
						action_list.push_back(potentialPlan[i]);
						generateFilter(environment);
					}

					total_plan_duration = planDuration;

				} else {
					ROS_INFO("Duration: %f, expected %f; plan discarded", planDuration, expectedPlanDuration);
				}
			}
		}
		planfile.close();
	}

	/**
	 * processes the parameters of a single PDDL action into an ActionDispatch message
	 */
	void PlanParser::processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params, PlanningEnvironment &environment) {

		// find the correct PDDL operator definition
		std::map<std::string,std::vector<std::string> >::iterator ait;
		ait = environment.domain_operators.find(msg.name);
		if(ait != environment.domain_operators.end()) {

			// add the PDDL parameters to the action dispatch
			for(size_t i=0; i<ait->second.size(); i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = ait->second[i];
				pair.value = params[i];
				msg.parameters.push_back(pair);

				// prepare object existence for the knowledge filter
				bool add = true;
				for(size_t j=0; j<filter_objects.size(); j++)
					if(0==filter_objects[j].compare(params[i])) add = false;
				if(add) filter_objects.push_back(params[i]);
			}

			// prepare object attributes for the knowledge filter
			for(size_t i=0; i<environment.domain_operator_precondition_map[msg.name].size(); i++) {
				std::vector<std::string> filterAttribute;
				std::vector<std::string> precondition = environment.domain_operator_precondition_map[msg.name][i];
				filterAttribute.push_back(precondition[0]);
				for(size_t j=1; j<precondition.size(); j++) {
					// label
					if(j>1) filterAttribute.push_back(precondition[j]);
					// instance name
					for(size_t k=0;k<ait->second.size();k++) {
						if(0==ait->second[k].compare(precondition[j]))
							filterAttribute.push_back(params[k]);
					}
				}
				filter_attributes.push_back(filterAttribute);
			}
		} // end of operator
	}

	/*-----------------*/
	/* Planning filter */
	/*-----------------*/

	/**
	 * populates the knowledge filter messages
	 */
	void PlanParser::generateFilter(PlanningEnvironment &environment) {

		knowledge_filter.clear();

		// populate filter message with objects
		for(size_t i=0; i<filter_objects.size(); i++) {
			rosplan_knowledge_msgs::KnowledgeItem filterItem;
			filterItem.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			filterItem.instance_type = environment.object_type_map[filter_objects[i]];
			filterItem.instance_name = filter_objects[i];
			knowledge_filter.push_back(filterItem);
		}

		// populate filter message with attributes
		// TODO only statics, not all preconditions.
		for(size_t i=0; i<filter_attributes.size(); i++) {
			rosplan_knowledge_msgs::KnowledgeItem filterItem;
			filterItem.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			filterItem.attribute_name = filter_attributes[i][0];
			if (filter_attributes[i].size() > 1)
			{
				filterItem.instance_type = environment.object_type_map[filter_attributes[i][1]];
				filterItem.instance_name = filter_attributes[i][1];
				for(size_t j=2; j<filter_attributes[i].size()-1; j+=2) {
					diagnostic_msgs::KeyValue pair;
					pair.key = filter_attributes[i][j];
					pair.value = filter_attributes[i][j+1];
					filterItem.values.push_back(pair);
				}
			} 
			knowledge_filter.push_back(filterItem);
		}
	}
} // close namespace
