#include "rosplan_planning_system/ProblemGeneration/PDDLProblemGenerator.h"

namespace KCL_rosplan {

	/**
	 * generates a PDDL problem file.
	 * This file is later read by the planner.
	 */
	void PDDLProblemGenerator::generatePDDLProblemFile(std::string &problemPath) {

		std::ofstream pFile;
		pFile.open((problemPath).c_str());

		makeHeader(pFile);
		makeInitialState(pFile);
		makeGoals(pFile);
	}

	/*--------*/
	/* header */
	/*--------*/

	void PDDLProblemGenerator::makeHeader(std::ofstream &pFile) {

		// setup service calls
		ros::NodeHandle nh;
		ros::ServiceClient getNameClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainNameService>("/kcl_rosplan/get_domain_name");
		ros::ServiceClient getTypesClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>("/kcl_rosplan/get_domain_types");
		ros::ServiceClient getInstancesClient = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");

		// get domain name
		rosplan_knowledge_msgs::GetDomainNameService nameSrv;
		if (!getNameClient.call(nameSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service /kcl_rosplan/get_domain_name");
		}

		pFile << "(define (problem task)" << std::endl;
		pFile << "(:domain " << nameSrv.response.domain_name << ")" << std::endl;

		/* objects */
		pFile << "(:objects" << std::endl;

		// get types
		rosplan_knowledge_msgs::GetDomainTypeService typeSrv;
		if (!getTypesClient.call(typeSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service /kcl_rosplan/get_domain_types");
		}

		// get instances of each type
		for(size_t t=0; t<typeSrv.response.types.size(); t++) {

			rosplan_knowledge_msgs::GetInstanceService instanceSrv;
			instanceSrv.request.type_name = typeSrv.response.types[t];

			if (!getInstancesClient.call(instanceSrv)) {
				ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service /kcl_rosplan/get_instances: %s", instanceSrv.request.type_name.c_str());
			} else {
				if(instanceSrv.response.instances.size() == 0) continue;
				pFile << "    ";
				for(size_t i=0;i<instanceSrv.response.instances.size();i++) {
					pFile << instanceSrv.response.instances[i] << " ";
				}
				pFile << "- " << typeSrv.response.types[t] << std::endl;
			}
		}

		pFile << ")" << std::endl;
	}

	/*---------------*/
	/* initial state */
	/*---------------*/

	void PDDLProblemGenerator::makeInitialState(std::ofstream &pFile) {

		ros::NodeHandle nh;
		ros::ServiceClient getDomainPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/kcl_rosplan/get_domain_predicates");
		ros::ServiceClient getDomainFuncsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/kcl_rosplan/get_domain_functions");
		ros::ServiceClient getAttrsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");

		pFile << "(:init" << std::endl;

		// get propositions
		rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;
		if (!getDomainPropsClient.call(domainAttrSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service /kcl_rosplan/get_domain_predicates");
		} else {

			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait = domainAttrSrv.response.items.begin();
			for(; ait != domainAttrSrv.response.items.end(); ait++) {

				rosplan_knowledge_msgs::GetAttributeService attrSrv;
				attrSrv.request.predicate_name = ait->name;
				if (!getAttrsClient.call(attrSrv)) {
					ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service /kcl_rosplan/get_current_knowledge %s", attrSrv.request.predicate_name.c_str());
				} else {
					if(attrSrv.response.attributes.size() == 0) continue;

					for(size_t i=0;i<attrSrv.response.attributes.size();i++) {
						rosplan_knowledge_msgs::KnowledgeItem attr = attrSrv.response.attributes[i];
						pFile << "    (";
						
						//Check if the attribute is negated
						if(attr.is_negative) pFile << "not (";

						pFile << attr.attribute_name;
						for(size_t j=0; j<attr.values.size(); j++) {
							pFile << " " << attr.values[j].value;
						}
						pFile << ")";
						if(attr.is_negative) pFile << ")";
						pFile << std::endl;
					}
				}
				pFile << std::endl;
			}
		}

		// get functions
		if (!getDomainFuncsClient.call(domainAttrSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service /kcl_rosplan/get_domain_functions");
		} else {

			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait = domainAttrSrv.response.items.begin();
			for(; ait != domainAttrSrv.response.items.end(); ait++) {

				rosplan_knowledge_msgs::GetAttributeService attrSrv;
				attrSrv.request.predicate_name = ait->name;
				if (!getAttrsClient.call(attrSrv)) {
					ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service /kcl_rosplan/get_current_knowledge %s", attrSrv.request.predicate_name.c_str());
				} else {
					if(attrSrv.response.attributes.size() == 0) continue;

					for(size_t i=0;i<attrSrv.response.attributes.size();i++) {
						rosplan_knowledge_msgs::KnowledgeItem attr = attrSrv.response.attributes[i];
						pFile << "    (= (";

						pFile << attr.attribute_name;
						for(size_t j=0; j<attr.values.size(); j++) {
							pFile << " " << attr.values[j].value;
						}
						pFile << ") " << attr.function_value << ")" << std::endl;
					}
				}
				pFile << std::endl;
			}
		}

		pFile << ")" << std::endl;
	}

	/*------*/
	/* goal */
	/*------*/

	void PDDLProblemGenerator::makeGoals(std::ofstream &pFile) {
			
		ros::NodeHandle nh;
		ros::ServiceClient getCurrentGoalsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_goals");

		pFile << "(:goal (and" << std::endl;

		// get current goals
		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		if (!getCurrentGoalsClient.call(currentGoalSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service /kcl_rosplan/get_current_goals");
		} else {

			for(size_t i=0;i<currentGoalSrv.response.attributes.size();i++) {
				rosplan_knowledge_msgs::KnowledgeItem attr = currentGoalSrv.response.attributes[i];
				if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {
					pFile << "    (" + attr.attribute_name;
					for(size_t j=0; j<attr.values.size(); j++) {
						pFile << " " << attr.values[j].value;
					}
					pFile << ")" << std::endl;
				}
			}
		}
		pFile << ")))" << std::endl;
	}
} // close namespace
