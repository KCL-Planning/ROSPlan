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

#ifndef KCL_ProblemGenerator
#define KCL_ProblemGenerator

namespace KCL_rosplan {

	class ProblemGenerator
	{
	protected:

		std::string domain_name_service;
		std::string domain_type_service;
		std::string domain_predicate_service;
		std::string domain_function_service;
		std::string domain_operator_details_service;
		std::string domain_operator_list_service;

		std::string state_instance_service;
		std::string state_proposition_service;
		std::string state_function_service;
		std::string state_timed_knowledge_service;
		std::string state_goal_service;
		std::string state_metric_service;

		virtual void makeProblem(std::ofstream &pFile) =0;

		// params
        std::string knowledge_base;
	public:

        ProblemGenerator(const std::string& kb);

		virtual void generateProblemFile(std::string &problemPath);

	};
} // close namespace

#endif
