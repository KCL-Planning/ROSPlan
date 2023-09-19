#include "rosplan_planning_system/PlannerInterface/SMTPlannerInterface.h"

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	SMTPlannerInterface::SMTPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
	}

	SMTPlannerInterface::~SMTPlannerInterface()
	{
		delete plan_server;
	}

	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 */
	bool SMTPlannerInterface::runPlanner() {

		// save problem to file for SMT
		saveProblem();

		// prepare the planner command line
		std::string commandString = prepareCommand();
		commandString = commandString + " > " + data_path + "plan.pddl";

		// call the planer
		callPlanner(commandString);

		// check the planner solved the problem
		std::ifstream planfile;
		planfile.open((data_path + "plan.pddl").c_str());
		std::string line;
		std::stringstream ss;

		int curr, next;
		bool solved = false;
		double planDuration;

		while (std::getline(planfile, line)) {

			if (line.find("0.0:", 0) != std::string::npos){
				solved = true;
			}
			planDuration = 0;
			ss.str("");
			while (std::getline(planfile, line)) {
				if (line.length()<2)
					break;
				ss << line << std::endl;
			}
			planner_output = ss.str();

		}
		planfile.close();

		solvedMessages(solved);

		return solved;
	}

} // close namespace
