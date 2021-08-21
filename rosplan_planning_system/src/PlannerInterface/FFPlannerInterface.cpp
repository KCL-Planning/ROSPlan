#include "rosplan_planning_system/PlannerInterface/FFPlannerInterface.h"

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	FFPlannerInterface::FFPlannerInterface(ros::NodeHandle& nh)
	{
		node_handle = &nh;

		plan_server = new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>((*node_handle), "start_planning", boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false);

		// publishing raw planner output
		std::string plannerTopic = "planner_output";
		node_handle->getParam("planner_topic", plannerTopic);
		plan_publisher = node_handle->advertise<std_msgs::String>(plannerTopic, 1, true);

		node_handle->param<bool>("use_ffha", this->use_ffha, false);

		// start planning action server
		plan_server->start();
	}

	FFPlannerInterface::~FFPlannerInterface()
	{
		delete plan_server;
	}

	/**
	 * Runs external commands
	 */
	std::string FFPlannerInterface::runCommand(std::string cmd) {
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
	bool FFPlannerInterface::runPlanner() {
		planner_output.clear(); // Clear previous plan
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

		bool solved = false;
		while (not solved and std::getline(planfile, line)) {
			// skip lines until there is a plan
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

		KCL_rosplan::FFPlannerInterface pi(nh);

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
