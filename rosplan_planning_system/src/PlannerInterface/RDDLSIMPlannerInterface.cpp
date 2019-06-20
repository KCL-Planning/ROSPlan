//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 28/09/18.
//
#include "rosplan_planning_system/PlannerInterface/RDDLSIMPlannerInterface.h"
#include <regex>

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	RDDLSIMPlannerInterface::RDDLSIMPlannerInterface(ros::NodeHandle& nh)
	{
		node_handle = &nh;

		plan_server = new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>((*node_handle), "start_planning", boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false);

		// publishing raw planner output
		std::string plannerTopic = "planner_output";
		node_handle->getParam("planner_topic", plannerTopic);
		plan_publisher = node_handle->advertise<std_msgs::String>(plannerTopic, 1, true);

		// start planning action server
		plan_server->start();
	}
	
	RDDLSIMPlannerInterface::~RDDLSIMPlannerInterface()
	{
		delete plan_server;
	}

	/**
	 * Runs external commands
	 */
	std::string RDDLSIMPlannerInterface::runCommand(std::string cmd) {
		std::string data;
		FILE *stream;
		char buffer[1000];
		stream = popen(cmd.c_str(), "r");
		while ( fgets(buffer, 1000, stream) != NULL )
			data.append(buffer);
		pclose(stream);
		return data;
	}

	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 */
	bool RDDLSIMPlannerInterface::runPlanner() {

		// save problem to file for planner
		if(use_problem_topic && problem_instance_received) {
			ROS_INFO("KCL: (%s) (%s) Writing problem to file.", ros::this_node::getName().c_str(), problem_name.c_str());
			std::ofstream dest;
			dest.open((problem_path).c_str());
			dest << problem_instance;
			dest.close();
		}

		// prepare the planner command line
		std::string str = planner_command;
		std::size_t dit = str.find("DOMAIN");
		if(dit!=std::string::npos) str.replace(dit,6,domain_path);
		std::size_t pit = str.find("PROBLEM");
		if(pit!=std::string::npos) str.replace(pit,7,problem_path);
		std::string commandString = str + " > " + data_path + "plan.pddl";

		// call the planer
		ROS_INFO("KCL: (%s) (%s) Running: %s", ros::this_node::getName().c_str(), problem_name.c_str(),  commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("KCL: (%s) (%s) Planning complete", ros::this_node::getName().c_str(), problem_name.c_str());

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
		if(!solved) ROS_INFO("KCL: (%s) (%s) Plan was unsolvable.", ros::this_node::getName().c_str(), problem_name.c_str());
		else ROS_INFO("KCL: (%s) (%s) Plan was solved.", ros::this_node::getName().c_str(), problem_name.c_str());

		return solved;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		srand (static_cast <unsigned> (time(0)));

		ros::init(argc,argv,"rosplan_planner_interface");
		ros::NodeHandle nh("~");

		KCL_rosplan::RDDLSIMPlannerInterface pi(nh);

		// subscribe to problem instance
		std::string problemTopic = "problem_instance";
		nh.getParam("problem_topic", problemTopic);
		ros::Subscriber problem_sub = nh.subscribe(problemTopic, 1, &KCL_rosplan::PlannerInterface::problemCallback, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

		// start the planning services
		ros::ServiceServer service1 = nh.advertiseService("planning_server", &KCL_rosplan::PlannerInterface::runPlanningServerDefault, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));
		ros::ServiceServer service2 = nh.advertiseService("planning_server_params", &KCL_rosplan::PlannerInterface::runPlanningServerParams, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
