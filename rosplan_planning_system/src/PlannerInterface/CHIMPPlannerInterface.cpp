#include "rosplan_planning_system/PlannerInterface/CHIMPPlannerInterface.h"

namespace KCL_rosplan {


	/*-------------*/
	/* constructor */
	/*-------------*/

	CHIMPPlannerInterface::CHIMPPlannerInterface(ros::NodeHandle& nh)
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
	
	CHIMPPlannerInterface::~CHIMPPlannerInterface()
	{
		delete plan_server;
	}

	/**
	 * Runs external commands
	 */
	std::string CHIMPPlannerInterface::runCommand(std::string cmd) {
		std::string data;
		FILE *stream;
		char buffer[1000];
		stream = popen(cmd.c_str(), "r");
		while ( fgets(buffer, 1000, stream) != NULL )
			data.append(buffer);
		pclose(stream);
		return data;
	}

	void CHIMPPlannerInterface::writeProblemToFile() {
		std::ofstream dest;
		dest.open((problem_path).c_str());
		dest << problem_instance;
		dest.close();
	}

	/**
	 * Creates command line string by setting domain and problem path and output.
	 */ 
	std::string CHIMPPlannerInterface::prepareCommand() {
		std::string str = planner_command;
		std::size_t dit = str.find("DOMAIN");
		if(dit!=std::string::npos) str.replace(dit,6,domain_path);
		std::size_t pit = str.find("PROBLEM");
		if(pit!=std::string::npos) str.replace(pit,7,problem_path);
		std::size_t oit = str.find("OUTPUT");
		if(oit!=std::string::npos) str.replace(oit,6,data_path + "plan.txt");
		std::size_t eit = str.find("ESTEREL");
		if(eit!=std::string::npos) str.replace(eit,7,data_path + "esterel.yaml");
				
		return str;
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
		if(use_problem_topic && problem_instance_received) {
			ROS_INFO("ROSPlan: (%s) (%s) Writing problem to file.", ros::this_node::getName().c_str(), problem_name.c_str());
			writeProblemToFile();
		}
		
		// call the planer
		std::string commandString = prepareCommand();
		ROS_INFO("ROSPlan: (%s) (%s) Running: %s", ros::this_node::getName().c_str(), problem_name.c_str(),  commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("ROSPlan: (%s) (%s) Planning complete", ros::this_node::getName().c_str(), problem_name.c_str());

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


		if(!solved) ROS_INFO("ROSPlan: (%s) (%s) Plan was unsolvable.", ros::this_node::getName().c_str(), problem_name.c_str());
		else ROS_INFO("ROSPlan: (%s) (%s) Plan was solved.", ros::this_node::getName().c_str(), problem_name.c_str());

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

		KCL_rosplan::CHIMPPlannerInterface pi(nh);
		
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
