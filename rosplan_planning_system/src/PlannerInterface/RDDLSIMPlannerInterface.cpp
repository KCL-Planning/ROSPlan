//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 28/09/18.
//
#include "rosplan_planning_system/PlannerInterface/RDDLSIMPlannerInterface.h"
#include <regex>

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	RDDLSIMPlannerInterface::RDDLSIMPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
	}
	
	RDDLSIMPlannerInterface::~RDDLSIMPlannerInterface()
	{
		delete plan_server;
	}

	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 */
	bool RDDLSIMPlannerInterface::runPlanner() {

		// save problem to file for planner
		saveProblem();

		// prepare the planner command line
		std::string commandString = prepareCommand() + " > " + data_path + "plan.pddl";

		// call the planer
		callPlanner(commandString);

		// CHECK PLANNER OUTPUT
        // Planner output example:
        //          ** Actions received: [move-current-dir(e1, e3);, open-door-going-down(e0);]
        //          ** Actions received: [close-door(e1);]
		// check the planner solved the problem
		std::ifstream planfile;
		planfile.open((data_path + "plan.pddl").c_str());
		std::string line;
		planner_output.clear();

        std::regex actionline_rgx("\\*\\* Actions received: \\[(.*)\\]"); // Action received line filter
        // Action name and parameters extraction: if more than one action, it my have a comma and a space.
        // Then we group the action name and the parameters, ignoring the parenthesis. if no parameters the second part is matched
        std::regex action_name_params_rgx(",?\\s?(.+)\\((.*)\\)|,?\\s?(.+)");
        std::smatch match;
        int action_idx = 0;
		while (std::getline(planfile, line)) {
		std::cout << "test: " << line << std::endl;
            // Check any action was found
            if (std::regex_search(line, match, actionline_rgx)) {
                std::istringstream action_list(match[1].str()); // Get all the actions in a sstream
                std::string action;
                while (getline(action_list, action, ';')) { // Get each action separately
                    if (std::regex_search(action, match, action_name_params_rgx)) { // Separate action name and parameters
                        // output_format:  0.000: (undock kenny wp1)  [10.000]\n
                        std::string action_name = (match[1].str().empty())? match[3].str(): match[1].str();
                        planner_output +=  std::to_string(action_idx) + ": (" + action_name; // Action name
                        std::istringstream p(match[2].str()); // parameters
                        std::string param;
                        while (getline(p, param, ',')) {
                        	if (param[0] != ' ') planner_output += " ";
                            planner_output += param;
                        }
                        planner_output += ")  [0.001]\n"; // Close parenthesis and add duration
                    }
                }
                ++action_idx;
            }
		}
		planfile.close();
        bool solved = !planner_output.empty();
		solvedMessages(solved);

		return solved;
	}

} // close namespace
