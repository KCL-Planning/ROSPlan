#include "rosplan_planning_system/PDDLProblemGenerator.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

namespace KCL_rosplan {

	/**
	 * generates a PDDL problem file.
	 * This file is later read by the planner.
	 */
	void PDDLProblemGenerator::generatePDDLProblemFile(PlanningEnvironment &environment, std::string &problemPath) {

		ROS_INFO("KCL: (PS) Generating PDDL problem file");
		std::ofstream pFile;
		pFile.open((problemPath).c_str());

		makeHeader(environment, pFile);
		makeInitialState(environment, pFile);
		makeGoals(environment, pFile);
	}

	/*--------*/
	/* header */
	/*--------*/

	void PDDLProblemGenerator::makeHeader(PlanningEnvironment environment, std::ofstream &pFile) {

		pFile << "(define (problem " << environment.domainName << "_task)" << std::endl;
		pFile << "(:domain " << environment.domainName << ")" << std::endl;

		/* objects */
		pFile << "(:objects" << std::endl;
		for (std::map<std::string,std::vector<std::string> >::iterator iit=environment.type_object_map.begin(); iit!=environment.type_object_map.end(); ++iit) {
			if(iit->second.size()>0) {
				pFile << "    ";
				for(size_t i=0;i<iit->second.size();i++) pFile << iit->second[i] << " ";
				pFile << "- " << iit->first << std::endl;
			}
		}
		pFile << ")" << std::endl;
	}

	/*---------------*/
	/* initial state */
	/*---------------*/

	void PDDLProblemGenerator::makeInitialState(PlanningEnvironment environment, std::ofstream &pFile) {

		pFile << "(:init" << std::endl;

		// add knowledge to the initial state
		for(size_t i=0; i<environment.domain_attributes.size(); i++) {
			
			std::stringstream ss;			
			ss << "    (";

			// output function equality
			if(environment.domain_attributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION) {
				ss << "= (";
			};

			ss << environment.domain_attributes[i].attribute_name;

			// fetch the corresponding symbols from domain
			std::map<std::string,std::vector<std::string> >::iterator ait;
			ait = environment.domain_predicates.find(environment.domain_attributes[i].attribute_name);
			if(ait == environment.domain_predicates.end())
				ait = environment.domain_functions.find(environment.domain_attributes[i].attribute_name);
			if(ait == environment.domain_functions.end())
				continue;

			// find the PDDL parameters in the KnowledgeItem
			bool writeAttribute = true;
			for(size_t j=0; j<ait->second.size(); j++) {
				bool found = false;
				for(size_t k=0; k<environment.domain_attributes[i].values.size(); k++) {
					if(0 == environment.domain_attributes[i].values[k].key.compare(ait->second[j])) {
						ss << " " << environment.domain_attributes[i].values[k].value;
						found = true;
					}
				}
				if(!found) writeAttribute = false; 
			};
			ss << ")";

			// output function value
			if(environment.domain_attributes[i].knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION) {
				ss << " " << environment.domain_attributes[i].function_value << ")";
			};

			if(writeAttribute) pFile << ss.str() << std::endl;
		}

		// add knowledge to the initial state
		for(size_t i=0; i<environment.instance_attributes.size(); i++) {

			std::stringstream ss;
			bool writeAttribute = false;
			
			// check if attribute is a PDDL predicate
			std::map<std::string,std::vector<std::string> >::iterator ait;
			ait = environment.domain_predicates.find(environment.instance_attributes[i].attribute_name);
			if(ait != environment.domain_predicates.end()) {
				writeAttribute = true;
				ss << "    (" + environment.instance_attributes[i].attribute_name;

				// find the PDDL parameters in the KnowledgeItem
				for(size_t j=0; j<ait->second.size(); j++) {
					bool found = false;
					for(size_t k=0; k<environment.instance_attributes[i].values.size(); k++) {
						if(0 == environment.instance_attributes[i].values[k].key.compare(ait->second[j])) {
							ss << " " << environment.instance_attributes[i].values[k].value;
							found = true;
						}
					}
					if(!found) writeAttribute = false;
				};
				ss << ")";
			}
			if(writeAttribute) pFile << ss.str() << std::endl;
		}
		pFile << ")" << std::endl;
	}

	/*-------*/
	/* goals */
	/*-------*/

	void PDDLProblemGenerator::makeGoals(PlanningEnvironment environment, std::ofstream &pFile) {

		pFile << "(:goal (and" << std::endl;
			
		// propositions in the initial state
		for(size_t i=0; i<environment.goal_attributes.size(); i++) {

			std::stringstream ss;
			bool writeAttribute = true;
			
			// check if attribute belongs in the PDDL model
			std::map<std::string,std::vector<std::string> >::iterator ait;
			ait = environment.domain_predicates.find(environment.goal_attributes[i].attribute_name);
			if(ait != environment.domain_predicates.end()) {

				ss << "    (" + environment.goal_attributes[i].attribute_name;

				// find the PDDL parameters in the KnowledgeItem
				bool found = false;
				for(size_t j=0; j<ait->second.size(); j++) {
				for(size_t k=0; k<environment.goal_attributes[i].values.size(); k++) {
					if(0 == environment.goal_attributes[i].values[k].key.compare(ait->second[j])) {
						ss << " " << environment.goal_attributes[i].values[k].value;
						found = true;
					}
				}};
				if(!found) writeAttribute = false;

				ss << ")";

			} else
				writeAttribute = false;

			if(writeAttribute) pFile << ss.str() << std::endl;
		}
		pFile << ")))" << std::endl;
	}
} // close namespace
