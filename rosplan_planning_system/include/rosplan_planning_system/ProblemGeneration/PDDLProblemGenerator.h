/**
 * This class is responsible for generating the PDDL instance.
 * This is done by using the objects requested from Knowedge services.
 */
#include "ros/ros.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>

#include "rosplan_knowledge_msgs/GetDomainNameService.h"
#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetMetricService.h"

#ifndef KCL_PDDLproblemgenerator
#define KCL_PDDLproblemgenerator

namespace KCL_rosplan {

	class PDDLProblemGenerator
	{
	private:

		std::string domain_name_service;
		std::string domain_type_service;
		std::string domain_predicate_service;
		std::string domain_function_service;

		std::string state_instance_service;
		std::string state_proposition_service;
		std::string state_function_service;
		std::string state_timed_knowledge_service;
		std::string state_goal_service;
		std::string state_metric_service;

		void makeHeader(std::ofstream &pFile);
		void makeInitialState(std::ofstream &pFile);
		void makeGoals(std::ofstream &pFile);
		void makeMetric(std::ofstream &pFile);
		void printExpression(std::ofstream &pFile, rosplan_knowledge_msgs::ExprComposite &e);
		void parseInequalityExpression(rosplan_knowledge_msgs::ExprComposite exprComposite, int size, std::ofstream &pFile);

	public:

		// params
		std::string knowledge_base;

		void generatePDDLProblemFile(std::string &problemPath);

	};
} // close namespace

#endif
