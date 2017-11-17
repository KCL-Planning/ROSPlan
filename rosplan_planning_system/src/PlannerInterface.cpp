#include "rosplan_planning_system/PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	PlannerInterface::PlannerInterface(ros::NodeHandle& nh)
		: plan_parser(new POPFEsterelPlanParser(nh)),
		  plan_server(new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>(nh_, "/kcl_rosplan/start_planning", boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false))
	{

		// publishing "plan", "problem"
		plan_publisher = nh.advertise<rosplan_dispatch_msgs::CompletePlan>("/kcl_rosplan/plan", 5, true);
		problem_publisher = nh.advertise<std_msgs::String>("/kcl_rosplan/problem", 5, true);

		// start planning action server
		plan_server->start();
	}
	
	PlannerInterface::~PlannerInterface()
	{
		delete plan_parser;
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

	/*--------------------*/
	/* problem generation */
	/*--------------------*/

	/**
	 * problem generation service with parameters.
	 */
	bool PlannerInterface::runProblemServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res) {
		return runProblemServer(req.domain_path, req.problem_path, req.data_path);
	}

	/**
	 * problem generation method.
	 */
	bool PlannerInterface::runProblemServer(std::string domainPath, std::string problemPath, std::string dataPath) {

		data_path = dataPath;
		domain_path = domainPath;
		problem_path = problemPath;
		
		// set problem name for ROS_INFO
		std::size_t lastDivide = problem_path.find_last_of("/\\");
		if(lastDivide != std::string::npos) {
			problem_name = problem_path.substr(lastDivide+1);
		} else {
			problem_name = problem_path;
		}

		ros::NodeHandle nh("~");

		// parse domain and update the environment from the knowledge base
		environment.parseDomain(domain_path);
		environment.update(nh);

		// generate PDDL problem
		pddl_problem_generator.generatePDDLProblemFile(environment, problem_path);
		ROS_INFO("KCL: (PlanningInterface) (%s) Problem generated.", problem_name.c_str());

		// publish problem
		std::ifstream problemIn(problem_path.c_str());
		if(problemIn) {
			std_msgs::String problemMsg;
			problemMsg.data = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
			problem_publisher.publish(problemMsg);
		}
		
		return true;
	}

	/*---------------- */
	/* Planning system */
	/*-----------------*/

	/**
	 * planning system service method (1) 
	 * loads parameters from param server
	 */
	bool PlannerInterface::runPlanningServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		ros::NodeHandle nh;

		// defaults
		data_path = "common/";
		domain_path = "common/domain.pddl";
		problem_path = "common/problem.pddl";
		planner_command = "timeout 60 common/bin/popf -n DOMAIN PROBLEM";

		// load params
		nh.param("/rosplan/domain_path", domain_path, std::string("common/domain.pddl"));
		nh.getParam("/rosplan_planning_system/data_path", data_path);
		nh.getParam("/rosplan_planning_system/problem_path", problem_path);
		nh.getParam("/rosplan_planning_system/planner_command", planner_command);

		// call planning server
		return runPlanningServer(domain_path, problem_path, data_path, planner_command);
	}

	/**
	 * planning system service method (2) 
	 * loads parameters from service request
	 */
	bool PlannerInterface::runPlanningServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res) {
		// call planning server
		return runPlanningServer(req.domain_path, req.problem_path, req.data_path, req.planner_command);
	}

	/**
	 * planning system service method (3) 
	 * loads parameters from actionlib goal
	 */
	void PlannerInterface::runPlanningServerAction(const rosplan_dispatch_msgs::PlanGoalConstPtr& goal) {
		// call planning server
		if(runPlanningServer(goal->domain_path, goal->problem_path, goal->data_path, goal->planner_command)) {
			plan_server->setSucceeded();
		} else {
			plan_server->setAborted();
		}
	}
	
	/**
	 * planning system; prepares planning; calls planner; parses plan.
	 */
	bool PlannerInterface::runPlanningServer(std::string domainPath, std::string problemPath, std::string dataPath, std::string plannerCommand) {

		// save parameters
		data_path = dataPath;
		domain_path = domainPath;
		problem_path = problemPath;
		planner_command = plannerCommand;
		
		// set problem name for ROS_INFO
		std::size_t lastDivide = problem_path.find_last_of("/\\");
		if(lastDivide != std::string::npos) {
			problem_name = problem_path.substr(lastDivide+1);
		} else {
			problem_name = problem_path;
		}

		ros::NodeHandle nh("~");

		// parse domain and update the environment from the ontology
		environment.parseDomain(domain_path);
		environment.update(nh);

		// generate PDDL problem
		pddl_problem_generator.generatePDDLProblemFile(environment, problem_path);
		ROS_INFO("KCL: (PlanningInterface) (%s) Problem generated.", problem_name.c_str());

		// publish problem
		std::ifstream problemIn(problem_path.c_str());
		if(problemIn) {
			std_msgs::String problemMsg;
			problemMsg.data = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
			problem_publisher.publish(problemMsg);
		}

		// run planner; generate a plan
		if(!runPlanner())
			return false;

		// publish plan
		rosplan_dispatch_msgs::CompletePlan planMsg;
		planMsg.plan = plan_parser->action_list;
		plan_publisher.publish(planMsg);
		
		return true;
	}

	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 */
	bool PlannerInterface::runPlanner() {

		// prepare the planner command line
		std::string str = planner_command;
		std::size_t dit = str.find("DOMAIN");
		if(dit!=std::string::npos) str.replace(dit,6,domain_path);
		std::size_t pit = str.find("PROBLEM");
		if(pit!=std::string::npos) str.replace(pit,7,problem_path);
		std::string commandString = str + " > " + data_path + "plan.pddl";

		// call the planer
		ROS_INFO("KCL: (PlannerInterface) (%s) Running: %s", problem_name.c_str(),  commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("KCL: (PlannerInterface) (%s) Planning complete", problem_name.c_str());

		// check the planner solved the problem
		std::ifstream planfile;
		planfile.open((data_path + "plan.pddl").c_str());
		std::string line;
		bool solved = false;
		
		while(!planfile.eof() && !solved) {
			getline(planfile, line);
			if (line.find("; Plan found", 0) != std::string::npos)
				solved = true;
			if (line.find("ff: found", 0) != std::string::npos)
				solved = true;
		}
		if(!solved) {
			planfile.close();
			ROS_INFO("KCL: (PlannerInterface) (%s) Plan was unsolvable.", problem_name.c_str());
			return false;
		}

		// save plan to file
		std::stringstream ss;
		std::ifstream source;
		std::ofstream dest;
		source.open((data_path + "plan.pddl").c_str());
		dest.open((data_path + "plan_" + ss.str()).c_str());
		dest << source.rdbuf();
		source.close();
		dest.close();

		// convert plan into message list
		plan_parser->preparePlan(data_path, environment, 0);

		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		srand (static_cast <unsigned> (time(0)));

		ros::init(argc,argv,"rosplan_planner_interface");
		ros::NodeHandle nh;

		KCL_rosplan::PlannerInterface PlannerInterface(nh);
		
		// start the planning services
		ros::ServiceServer service1 = nh.advertiseService("/kcl_rosplan/planning_server", &KCL_rosplan::PlannerInterface::runPlanningServerDefault, &PlannerInterface);
		ros::ServiceServer service2 = nh.advertiseService("/kcl_rosplan/planning_server_params", &KCL_rosplan::PlannerInterface::runPlanningServerParams, &PlannerInterface);
		ros::ServiceServer service4 = nh.advertiseService("/kcl_rosplan/problem_server_params", &KCL_rosplan::PlannerInterface::runProblemServerParams, &PlannerInterface);

		ROS_INFO("KCL: (PlannerInterface) Ready to receive");
		ros::spin();

		return 0;
	}
