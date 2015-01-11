/**
 * This class is responsible for generating the PDDL instance.
 * This is done by using the objects already retrieved and stored (see PlanningEnvironment.h).
 */
#include <string>
#include "PlanningEnvironment.h"

#ifndef KCL_PDDLproblemgenerator
#define KCL_PDDLproblemgenerator

namespace KCL_rosplan {

	class PDDLProblemGenerator
	{
	private:

		void makeHeader(PlanningEnvironment environment, std::ofstream &pFile);
		void makeInitialState(PlanningEnvironment environment, std::ofstream &pFile);
		void makeGoals(PlanningEnvironment environment, std::ofstream &pFile);

	public:

		void generatePDDLProblemFile(PlanningEnvironment &environment, std::string &problemPath);

	};
} // close namespace

#endif
