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

		void makeHeader(std::ofstream &pFile);
		void makeInitialState(std::ofstream &pFile);
		void makeGoals(std::ofstream &pFile);
		void makeMetric(std::ofstream &pFile);

	public:

		void generatePDDLProblemFile(std::string &problemPath);

	};
} // close namespace

#endif
