
#include <rosplan_planning_system/ProblemGeneration/ProblemGenerator.h>

#include "rosplan_planning_system/ProblemGeneration/ProblemGenerator.h"

namespace KCL_rosplan {

	/**
	 * generates a PDDL problem file.
	 * This file is later read by the planner.
	 */
	void ProblemGenerator::generateProblemFile(std::string &problemPath) {
	    if (knowledge_base.size() == 0) ROS_ERROR("KCL: (%s) Knowledge base is not set!", ros::this_node::getName().c_str());
		std::ofstream pFile;
		pFile.open((problemPath).c_str());
		makeProblem(pFile);
		pFile.close();
	}

	ProblemGenerator::ProblemGenerator(const std::string &kb) {
		knowledge_base = kb;

		std::stringstream ss;

		ss << "/" << knowledge_base << "/domain/name";
		domain_name_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/domain/types";
		domain_type_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/domain/predicates";
		domain_predicate_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/domain/operators";
        domain_operator_list_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/domain/operator_details";
		domain_operator_details_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/domain/functions";
		domain_function_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/state/instances";
		state_instance_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/state/propositions";
		state_proposition_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/state/functions";
		state_function_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/state/timed_knowledge";
		state_timed_knowledge_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/state/goals";
		state_goal_service = ss.str();
		ss.str("");

		ss << "/" << knowledge_base << "/state/metric";
		state_metric_service = ss.str();
		ss.str("");
	}

} // close namespace
