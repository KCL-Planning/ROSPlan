/**
 * This file parses the output of popf and generates a list of ActionDispatch messages.
 * It can be replaced or extended to add domain-specific post-processing of PDDL actions, or to
 * change parsing to another planner.
 */
#include "planning_dispatch_msgs/ActionDispatch.h"

#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

namespace KCL_rosplan
{

	/**
	 * populates the knowledge filter messages
	 */
	void processFilter() {

		KCL_rosplan::knowledgeFilter.clear();

		// populate filter message with objects
		for(size_t i=0; i<KCL_rosplan::filterObjects.size(); i++) {
			planning_knowledge_msgs::KnowledgeItem filterItem;
			filterItem.knowledge_type = planning_knowledge_msgs::KnowledgeItem::INSTANCE;
			filterItem.instance_type = KCL_rosplan::objectTypeMap[KCL_rosplan::filterObjects[i]];
			filterItem.instance_name = KCL_rosplan::filterObjects[i];
			KCL_rosplan::knowledgeFilter.push_back(filterItem);
		}

		// populate filter message with attributes
		// TODO ONLY STATICS! Not all preconditions.
		for(size_t i=0; i<KCL_rosplan::filterAttributes.size(); i++) {
			planning_knowledge_msgs::KnowledgeItem filterItem;
			filterItem.knowledge_type = planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
			filterItem.attribute_name = KCL_rosplan::filterAttributes[i][0];
			filterItem.instance_type = KCL_rosplan::objectTypeMap[KCL_rosplan::filterAttributes[i][1]];
			filterItem.instance_name = KCL_rosplan::filterAttributes[i][1];
			for(size_t j=2; j<KCL_rosplan::filterAttributes[i].size()-1; j+=2) {
				diagnostic_msgs::KeyValue pair;
				pair.key = KCL_rosplan::filterAttributes[i][j];
				pair.value = KCL_rosplan::filterAttributes[i][j+1];
				filterItem.values.push_back(pair);
			}
			KCL_rosplan::knowledgeFilter.push_back(filterItem);
		}
	}

	/**
	 * processes the parameters of a single PDDL action into an ActionDispatch message
	 */
	void processParameters(planning_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params) {

		// find the correct PDDL operator definition
		std::map<std::string,std::vector<std::string> >::iterator ait;
		ait = domainOperators.find(msg.name);
		if(ait != domainOperators.end()) {

			// add the PDDL parameters to the action dispatch
			for(size_t i=0; i<ait->second.size(); i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = ait->second[i];
				pair.value = params[i];
				msg.parameters.push_back(pair);

				// prepare object existence for the knowledge filter
				bool add = true;
				for(size_t j=0; j<filterObjects.size(); j++)
					if(0==KCL_rosplan::filterObjects[j].compare(params[i])) add = false;
				if(add) KCL_rosplan::filterObjects.push_back(params[i]);
			}

			// prepare object attributes for the knowledge filter
			for(size_t i=0; i<KCL_rosplan::domainOeratorPreconditionMap[msg.name].size(); i++) {
				std::vector<std::string> filterAttribute;
				std::vector<std::string> precondition = KCL_rosplan::domainOeratorPreconditionMap[msg.name][i];
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
				KCL_rosplan::filterAttributes.push_back(filterAttribute);
			}

			// add non-PDDL knowledge items to action dispatch
			std::map<std::string,std::vector<std::string> >::iterator pit;
			for(size_t i=0; i<params.size(); i++) {
				pit = domainPredicates.find(ait->second[i]);
				for(size_t j=0; j<KCL_rosplan::instanceAttributes.size(); j++) {
					// TODO have knowledge items be passed more cleanly in dispatch
					if(0==KCL_rosplan::instanceAttributes[j].instance_name.compare(params[i])) {
						for(size_t k=0; k<instanceAttributes[j].values.size(); k++) {
							diagnostic_msgs::KeyValue pair;
							pair.key = instanceAttributes[j].instance_name + "_" + instanceAttributes[j].attribute_name + "_" + instanceAttributes[j].values[k].key;
							pair.value = instanceAttributes[j].values[k].value;
							msg.parameters.push_back(pair);
						}
					}
				}
			}
		} // end of operator
	}

	/**
	 * parses the output of popf, generating a list of action messages.
	 */
	void preparePlan(std::string &dataPath)
	{
		// popf output
		std::ifstream planfile;
		planfile.open((dataPath + "plan.pddl").c_str());
		
		int curr, next; 
		std::string line;
		std::vector<planning_dispatch_msgs::ActionDispatch> potentialPlan;
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
				KCL_rosplan::freeActionID = KCL_rosplan::currentAction;
				planDuration = 0;

				while(!planfile.eof() && line.compare("")!=0) {

					getline(planfile, line);
					if (line.length()<2)
						break;

					planning_dispatch_msgs::ActionDispatch msg;

					// action ID
					msg.action_id = KCL_rosplan::freeActionID;
					KCL_rosplan::freeActionID++;

					// name
					curr=line.find("(")+1;
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
						std::string param = name_map[line.substr(at,curr-at)];
						params.push_back(param);
						++curr;
						at = curr;
					}
					processParameters(msg, params);

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
					while(KCL_rosplan::actionList.size() > KCL_rosplan::currentAction) {
						KCL_rosplan::actionList.pop_back();
					}

					// save better optimised plan
					for(size_t i=0;i<potentialPlan.size();i++) {
						KCL_rosplan::actionList.push_back(potentialPlan[i]);
						KCL_rosplan::processFilter();
					}

					KCL_rosplan::totalPlanDuration = planDuration;

				} else {
					ROS_INFO("Duration: %f, expected %f; plan discarded", planDuration, expectedPlanDuration);
				}
			}
		}
		planfile.close();
	}
} // close namespace
