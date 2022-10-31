#include "rosplan_planning_system/PlannerInterface/PPRPPlannerInterface.h"
#include <regex>

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

    PPRPPlannerInterface::PPRPPlannerInterface(ros::NodeHandle& nh)
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

    PPRPPlannerInterface::~PPRPPlannerInterface()
	{
		delete plan_server;
	}

	/**
	 * Runs external commands
	 */
	std::string PPRPPlannerInterface::runCommand(std::string cmd) {
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
	bool PPRPPlannerInterface::runPlanner() {

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

		if (!solved) ROS_WARN("KCL: (%s) (%s) Plan was unsolvable.", ros::this_node::getName().c_str(), problem_name.c_str());
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

		KCL_rosplan::PPRPPlannerInterface pi(nh);

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
