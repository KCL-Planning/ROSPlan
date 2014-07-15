/**
 * This file is responsible for generating the PDDL instance.
 * This is done by using the objects already retrieved and stored (see PlanningEnvironment.h).
 */
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

namespace KCL_rosplan {

	/**
	 * generates a PDDL problem file.
	 * This file is later read by the planner, and the result saved in dataPath/plan.pddl.
	 */
	void generatePDDLProblemFile(std::string &problemPath)
	{
		/*--------*/
		/* header */
		/*--------*/

		ROS_INFO("KCL: Generating PDDL problem file");
		std::ofstream pFile;
		pFile.open((problemPath).c_str());

		pFile << "(define (problem " << KCL_rosplan::domainName << "task)" << std::endl;
		pFile << "(:domain " << KCL_rosplan::domainName << ")" << std::endl;

		/* objects */
		pFile << "(:objects" << std::endl;
		for (std::map<std::string,std::vector<std::string> >::iterator iit=KCL_rosplan::typeObjectMap.begin();
				iit!=KCL_rosplan::typeObjectMap.end(); ++iit) {
			if(iit->second.size()>0) {
				pFile << "    ";
				for(size_t i=0;i<iit->second.size();i++) pFile << iit->second[i] << " ";
				pFile << "- " << iit->first << std::endl;
			}
		}
		pFile << ")" << std::endl;

		/*---------------*/
		/* initial state */
		/*---------------*/

		pFile << "(:init" << std::endl;

		// add knowledge to the initial state
		for(size_t i=0; i<domainAttributes.size(); i++) {
			
			std::stringstream ss;			
			ss << "    (" + domainAttributes[i].attribute_name;

			// fetch the corresponding symbols from domain
			std::map<std::string,std::vector<std::string> >::iterator ait;
			ait = domainPredicates.find(domainAttributes[i].attribute_name);
			if(ait == domainPredicates.end()) continue;

			// find the PDDL parameters in the KnowledgeItem
			bool writeAttribute = true;
			for(size_t j=0; j<ait->second.size(); j++) {
				bool found = false;
				for(size_t k=0; k<domainAttributes[i].values.size(); k++) {
					if(0 == domainAttributes[i].values[k].key.compare(ait->second[j])) {
						ss << " " << domainAttributes[i].values[k].value;
						found = true;
					}
				}
				if(!found) writeAttribute = false;
			};
			ss << ")";
			if(writeAttribute) pFile << ss.str() << std::endl;
		}

		// add knowledge to the initial state
		for(size_t i=0; i<instanceAttributes.size(); i++) {

			std::stringstream ss;
			bool writeAttribute = false;
			
			// check if attribute is a PDDL predicate
			std::map<std::string,std::vector<std::string> >::iterator ait;
			ait = domainPredicates.find(instanceAttributes[i].attribute_name);
			if(ait != domainPredicates.end()) {
				writeAttribute = true;
				ss << "    (" + instanceAttributes[i].attribute_name;

				// find the PDDL parameters in the KnowledgeItem
				for(size_t j=0; j<ait->second.size(); j++) {
					bool found = false;
					for(size_t k=0; k<instanceAttributes[i].values.size(); k++) {
						if(0 == instanceAttributes[i].values[k].key.compare(ait->second[j])) {
							ss << " " << instanceAttributes[i].values[k].value;
							found = true;
						}
					}
					if(!found) writeAttribute = false;
				};
				ss << ")";
			}

			// check if attribute is a PDDL function
			std::map<std::string,std::vector<std::string> >::iterator fit;
			fit = domainFunctions.find(instanceAttributes[i].attribute_name);
			if(fit != domainFunctions.end()) {

				writeAttribute = true;
				ss << "    (= (" + instanceAttributes[i].attribute_name;

				// find the PDDL parameters in the KnowledgeItem
				for(size_t j=0; j<fit->second.size(); j++) {
					bool found = false;
					for(size_t k=0; k<instanceAttributes[i].values.size(); k++) {
						if(0 == instanceAttributes[i].values[k].key.compare(fit->second[j])) {
							ss << " " << instanceAttributes[i].values[k].value;
							found = true;
						}
					}
					if(!found) writeAttribute = false;
				};
				ss << ") ";

				// find the PDDL function value in the KnowledgeItem
				bool found = false;
				for(size_t k=0; k<instanceAttributes[i].values.size(); k++) {
					if(0 == instanceAttributes[i].values[k].key.compare("function_value")) {
						ss << instanceAttributes[i].values[k].value;
						found = true;
					}
				}
				if(!found) writeAttribute = false;
				ss << ")";
			}

			if(writeAttribute) pFile << ss.str() << std::endl;
		}
		pFile << ")" << std::endl;

		/*-------*/
		/* goals */
		/*-------*/

		pFile << "(:goal (and" << std::endl;
			
		// propositions in the initial state
		for(size_t i=0; i<goalAttributes.size(); i++) {

			std::stringstream ss;
			bool writeAttribute = true;
			
			// check if attribute belongs in the PDDL model
			std::map<std::string,std::vector<std::string> >::iterator ait;
			ait = domainPredicates.find(goalAttributes[i].attribute_name);
			if(ait != domainPredicates.end()) {

				ss << "    (" + goalAttributes[i].attribute_name;

				// find the PDDL parameters in the KnowledgeItem
				bool found = false;
				for(size_t j=0; j<ait->second.size(); j++) {
				for(size_t k=0; k<goalAttributes[i].values.size(); k++) {
					if(0 == goalAttributes[i].values[k].key.compare(ait->second[j])) {
						ss << " " << goalAttributes[i].values[k].value;
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
