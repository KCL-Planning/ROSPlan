#include "rosplan_planning_system/PlannerInterface/PlannerInterface.h"

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	PlannerInterface::PlannerInterface(ros::NodeHandle& nh) {
		node_handle = &nh;

		plan_server = new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>((*node_handle), "start_planning", boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false);

		// publishing raw planner output
		std::string plannerTopic = "planner_output";
		node_handle->getParam("planner_topic", plannerTopic);
		plan_publisher = node_handle->advertise<std_msgs::String>(plannerTopic, 1, true);

		// start planning action server
		plan_server->start();
	}

	PlannerInterface::~PlannerInterface(){
		delete plan_server;
	}

	/**
     	* Runs external commands
     	*/
	std::string PlannerInterface::runCommand(std::string cmd) {
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

	void PlannerInterface::saveProblem(){	
		if(use_problem_topic && problem_instance_received) {
			ROS_INFO("ROSPlan: (%s) (%s) Writing problem to file.", ros::this_node::getName().c_str(), problem_name.c_str());
			std::ofstream dest;
			dest.open((problem_path).c_str());
			dest << problem_instance;
			dest.close();
		}
	}

	std::string PlannerInterface::prepareCommand() {
		std::string str = planner_command;
		std::size_t dit = str.find("DOMAIN");
		if(dit!=std::string::npos) str.replace(dit,6,domain_path);
		std::size_t pit = str.find("PROBLEM");
		if(pit!=std::string::npos) str.replace(pit,7,problem_path);

		return str;
	}

	void PlannerInterface::callPlanner(std::string & commandString){	
		ROS_INFO("ROSPlan: (%s) (%s) Running: %s", ros::this_node::getName().c_str(), problem_name.c_str(),  commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("ROSPlan: (%s) (%s) Planning complete", ros::this_node::getName().c_str(), problem_name.c_str());
	}

	void PlannerInterface::solvedMessages(bool & solved){
		if(!solved) ROS_INFO("ROSPlan: (%s) (%s) Plan was unsolvable.", ros::this_node::getName().c_str(), problem_name.c_str());
		else ROS_INFO("ROSPlan: (%s) (%s) Plan was solved.", ros::this_node::getName().c_str(), problem_name.c_str());	
	}

	/*----------------------*/
	/* Problem subscription */
	/*----------------------*/

	void PlannerInterface::problemCallback(const std_msgs::String& problemInstance) {
		ROS_INFO("ROSPlan: (%s) Problem received.", ros::this_node::getName().c_str());
		problem_instance_received = true;
		problem_instance_time = ros::WallTime::now().toSec();
		problem_instance = problemInstance.data;
	}

	/*--------------------*/
	/* Planning interface */
	/*--------------------*/

	/**
	 * planning system service method (1) 
	 * loads parameters from param server
	 */
	bool PlannerInterface::runPlanningServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		// defaults
		use_problem_topic = false;
		data_path = "common/";
		domain_path = "common/domain.pddl";
		problem_path = "common/problem.pddl";
		planner_command = "timeout 60 common/bin/popf -n DOMAIN PROBLEM";

		// load params
		node_handle->getParam("use_problem_topic", use_problem_topic);
		node_handle->getParam("domain_path", domain_path);
		node_handle->getParam("data_path", data_path);
		node_handle->getParam("problem_path", problem_path);
		node_handle->getParam("planner_command", planner_command);

		// call planning server
		return runPlanningServer(domain_path, problem_path, data_path, planner_command, use_problem_topic);
	}

	/**
	 * planning system service method (2) 
	 * loads parameters from service request
	 */
	bool PlannerInterface::runPlanningServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res) {
		// call planning server
		res.plan_found = runPlanningServer(req.domain_path, req.problem_path, req.data_path, req.planner_command, req.use_problem_topic);
		return true;
	}

	/**
	 * planning system service method (3) 
	 * loads parameters from actionlib goal
	 */
	void PlannerInterface::runPlanningServerAction(const rosplan_dispatch_msgs::PlanGoalConstPtr& goal) {
		// call planning server
		if(runPlanningServer(goal->domain_path, goal->problem_path, goal->data_path, goal->planner_command, goal->use_problem_topic)) {
			plan_server->setSucceeded();
		} else {
			plan_server->setAborted();
		}
	}
	
	/**
	 * planning system; prepares planning; calls planner; parses plan.
	 */
	bool PlannerInterface::runPlanningServer(std::string domainPath, std::string problemPath, std::string dataPath, std::string plannerCommand, bool useProblemTopic) {

		// save parameters
		data_path = dataPath;
		domain_path = domainPath;
		problem_path = problemPath;
		planner_command = plannerCommand;
		use_problem_topic = useProblemTopic;

        // check if data_path ends in "/" and add "/" if not
        const char *last_char = &data_path.back();
        if (strcmp(last_char,"/") != 0)data_path = data_path + "/";
		
		// set problem name for ROS_INFO
		std::size_t lastDivide = problem_path.find_last_of("/\\");
		if(lastDivide != std::string::npos) {
			problem_name = problem_path.substr(lastDivide+1);
		} else {
			problem_name = problem_path;
		}

		if(use_problem_topic && !problem_instance_received) {
			ROS_INFO("ROSPlan: (%s) (%s) Problem was not published yet.", ros::this_node::getName().c_str(), problem_name.c_str());
			return false;
		}

		bool success = runPlanner();

		// publish planner output
		if(success) {
			std_msgs::String planMsg;
			planMsg.data = planner_output;
			plan_publisher.publish(planMsg);
		}

		return success;
	}

} // close namespace
