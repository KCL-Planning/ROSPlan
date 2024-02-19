#include "rosplan_planning_system/PlannerInterface/CHIMPPlannerInterface.h"

namespace KCL_rosplan {


	/*-------------*/
	/* constructor */
	/*-------------*/

	CHIMPPlannerInterface::CHIMPPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
	}
	
	CHIMPPlannerInterface::~CHIMPPlannerInterface()
	{
		delete plan_server;
	}

	bool containsEsterel(const std::string& commandStr) {
		std::size_t eit = commandStr.find("ESTEREL");
		return eit!=std::string::npos;
	}

	std::string readEsterel(const std::string& esterelPath) {
		std::ifstream esterelFile;
		esterelFile.open(esterelPath.c_str());
		std::stringstream ss;
    	ss << esterelFile.rdbuf();
		esterelFile.close();
		return ss.str();
	}

	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 */
	bool CHIMPPlannerInterface::runPlanner() {

		// save problem to file for CHIMP
		saveProblem();
		
		// prepare the planner command line
		std::string commandString = prepareCommand();
		std::size_t oit = commandString.find("OUTPUT");
		if(oit!=std::string::npos) commandString.replace(oit,6,data_path + "plan.txt");
		std::size_t eit = commandString.find("ESTEREL");
		if(eit!=std::string::npos) commandString.replace(eit,7,data_path + "esterel.yaml");

		// call the planer
		callPlanner(commandString);

		// check if the planner solved the problem
		std::ifstream planfile;
		std::string planFilePath = data_path + "plan.txt";
		planfile.open(planFilePath.c_str());
		std::string line;

		// parse the plan file
		bool solved = false;
		std::string actions = "";
		while (std::getline(planfile, line)) {

			if (line.find("; Solution found", 0) != std::string::npos) {
				solved = true;
			} else if (line.find("; Actions:", 0) == std::string::npos) {
				// consume useless lines
			} else {
				std::stringstream ss;
				ss.str("");
				std::string line;
				while (std::getline(planfile, line)) {
					if (line.length()<2)
						break;
					ss << line << std::endl;
				}
				actions = ss.str();

				// ROS_INFO("ROSPlan: PlannerOutput: (%s)", planner_output.c_str());
			}
		}
		planfile.close();

		// if esterel is generated, parse it
		if(containsEsterel(planner_command)) {
			planner_output = readEsterel(data_path + "esterel.yaml");
		} else {
			planner_output = actions;
		}


		solvedMessages(solved);

		return solved;
	}

} // close namespace
