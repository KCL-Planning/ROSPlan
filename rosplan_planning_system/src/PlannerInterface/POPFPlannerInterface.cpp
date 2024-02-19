#include "rosplan_planning_system/PlannerInterface/POPFPlannerInterface.h"

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	POPFPlannerInterface::POPFPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
	}
	
	POPFPlannerInterface::~POPFPlannerInterface()
	{
		delete plan_server;
	}

	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 */
	bool POPFPlannerInterface::runPlanner() {

		// save problem to file for POPF
		saveProblem();

		// prepare the planner command line
		std::string commandString = prepareCommand() + " > " + data_path + "plan.pddl";

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

			if (line.find("; Plan found", 0) != std::string::npos || line.find(";;;; Solution Found", 0) != std::string::npos) {
				solved = true;
			} else if (line.find("; Time", 0) == std::string::npos) {
				// consume useless lines
			} else {
				// read a plan (might not be the last plan)
				planDuration = 0;
				ss.str("");
				while (std::getline(planfile, line)) {
					if (line.length()<2)
						break;
					ss << line << std::endl;
				}
				planner_output = ss.str();
			}
		}
		planfile.close();

		solvedMessages(solved);

		return solved;
	}

} // close namespace
