#include "rosplan_planning_system/PlannerInterface/FFPlannerInterface.h"

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	FFPlannerInterface::FFPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
		node_handle->param<bool>("use_ffha", this->use_ffha, false);
	}

	FFPlannerInterface::~FFPlannerInterface()
	{
		delete plan_server;
	}


	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 */
	bool FFPlannerInterface::runPlanner() {
		planner_output.clear(); // Clear previous plan
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

		bool solved = false;
		while (not solved and std::getline(planfile, line)) {
			// skip lines until there is a plan
			std::cout << line << std::endl;
			if (line.find("ff: found legal plan") != line.npos) {
				solved = true;
			}
		}

		// Parse the solved plan
		if (solved) {
			if (this->use_ffha)
			{
				std::getline(planfile, line); //go to first action line
			} else {
			// actions look like this:
			// step    0: got_place C1
			//         1: find_object V1 C1
			// plan cost: XX
				while (std::getline(planfile, line)) { // Move to the beginning of the plan
					if (line.substr(0, 4) == "step") {
						line = line.substr(4); // Remove the step
						break;
					}
				}
			}

			// First iteration line will be like   0: got_place C1
			while (line.find("plan cost") == line.npos and line.find("Total cost") == line.npos and line.find("time spend") == line.npos and line.size() > 0) {
				std::stringstream ss(line); // To trim whitespaces
				std::string aux;
				// Read the action number X:
				ss >> aux;
				std::string add = (use_ffha) ? " " : " (";
				planner_output += aux + add; // Add parenthesis before the action
				while (ss >> aux) { // Read the rest
					planner_output += aux;
					if (ss.good()) planner_output += " "; // Add a whitespace unless we have processed all the line
				}
				add = (use_ffha) ? "  [0.001]\n" : ")  [0.001]\n";
				planner_output += add; // Close parenthesis and add duration
				std::getline(planfile, line);
			}
			// Convert to lowercase as FF prints all the actions and parameters in uppercase
			std::transform(planner_output.begin(), planner_output.end(), planner_output.begin(), ::tolower);
		}
		planfile.close();

		solvedMessages(solved);

		return solved;
	}

} // close namespace
