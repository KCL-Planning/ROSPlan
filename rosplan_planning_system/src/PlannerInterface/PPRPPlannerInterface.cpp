#include "rosplan_planning_system/PlannerInterface/PPRPPlannerInterface.h"
#include <regex>

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

    PPRPPlannerInterface::PPRPPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
	}

    PPRPPlannerInterface::~PPRPPlannerInterface()
	{
		delete plan_server;
	}

	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 */
	bool PPRPPlannerInterface::runPlanner() {

		// save problem to file for planner
		saveProblem();
		// prepare the planner command line
		std::string commandString = prepareCommand() + " > " + data_path + "plan.pddl";

		// call the planer
		callPlanner(commandString);

		// check the planner solved the problem
		std::ifstream planfile;
		planfile.open((data_path + "plan.pddl").c_str());
		std::string line;

		// Planner output:
		// (...)
		//    Dead ends: 0 state(s). (0 recorded)
        //    action param1 param2 ... (X)
        //    (...)
        //    Plan length: Y step(s).

		bool solved = false;
		while (not solved and std::getline(planfile, line)) {
            if (line.find("No solution") != line.npos) break; // No plan found!
            if (line.find("Dead ends") != line.npos) solved = true; // Plan found!
		}

		// Parse plan
		if (solved) {
		    int idx = 0;
            //std::regex actionline_rgx("(.*)_DETDUP_.*_\\d+(.*?)$"); // Stochastic actions have the form goto_waypoint_DETDUP_0_WEIGHT_7 p1 p2 p3 (X)
            std::regex actionline_rgx("_DETDUP_.*_\\d+"); // Stochastic actions have the form goto_waypoint_DETDUP_0_WEIGHT_7 p1 p2 p3 (X)
            std::regex tr_ws("^ +| +$|( ) +"); // Trailing and extra whitespaces
		    while (std::getline(planfile, line) and line.find("Plan length") == line.npos) {
                planner_output += std::to_string(idx) + ": (";
                ++idx;
                line = line.substr(0, line.find(" (")); // This is the action with the parameters

                /*std::smatch match;
                if (std::regex_search(line, match, actionline_rgx)) {
                    line = match[1].str() +  match[2].str();
                }*/
                line = std::regex_replace(line, actionline_rgx, ""); // Remove trailing whitespaces
                line = std::regex_replace(line, tr_ws, "$1"); // Remove trailing whitespaces

                planner_output += line;
                planner_output += ")  [0.001]\n"; // Close parenthesis and add duration
		    }
		}

		planfile.close();

		solvedMessages(solved);

		return solved;
	}

} // close namespace
