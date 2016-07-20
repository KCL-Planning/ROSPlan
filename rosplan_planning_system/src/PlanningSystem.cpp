#include "rosplan_planning_system/PlanningSystem.h"
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

	PlanningSystem::PlanningSystem(ros::NodeHandle& nh)
		: system_status(READY),
		  plan_parser(new POPFEsterelPlanParser(nh)),
		  plan_server(new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>(nh_, "/kcl_rosplan/start_planning", boost::bind(&PlanningSystem::runPlanningServerAction, this, _1), false))
	{
		// dispatcher
		plan_dispatcher = new EsterelPlanDispatcher(*dynamic_cast<POPFEsterelPlanParser*>(plan_parser));

		// publishing system_state
		state_publisher = nh.advertise<std_msgs::String>("/kcl_rosplan/system_state", 5, true);

		// publishing "action_dispatch", "action_feedback", "plan"; listening "action_feedback"
		plan_publisher = nh.advertise<rosplan_dispatch_msgs::CompletePlan>("/kcl_rosplan/plan", 5, true);
		problem_publisher = nh.advertise<std_msgs::String>("/kcl_rosplan/problem", 5, true);
		plan_dispatcher->action_publisher = nh.advertise<rosplan_dispatch_msgs::ActionDispatch>("/kcl_rosplan/action_dispatch", 1000, true);
		plan_dispatcher->action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 5, true);

		// problem generation client
		generate_problem_client = nh.serviceClient<rosplan_knowledge_msgs::GenerateProblemService>("/kcl_rosplan/generate_planning_problem");

		// start planning action server
		plan_server->start();
	}
	
	PlanningSystem::~PlanningSystem()
	{
		delete plan_parser;
		delete plan_dispatcher;
		delete plan_server;
	}

	/**
	 * Runs external commands
	 */
	std::string PlanningSystem::runCommand(std::string cmd) {
		std::string data;
		FILE *stream;
		char buffer[1000];
		stream = popen(cmd.c_str(), "r");
		while ( fgets(buffer, 1000, stream) != NULL )
			data.append(buffer);
		pclose(stream);
		return data;
	}

	/*----------------------------*/
	/* PDDL2.1 problem generation */
	/*----------------------------*/

	/* problem service hook with parameters */
	bool PlanningSystem::runProblemServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res) {
		return runProblemServer(req.domain_path, req.problem_path, req.data_path);
	}

	/**
	 * planning system service method; prepares planning; main loop.
	 */
	bool PlanningSystem::runProblemServer(std::string domainPath, std::string problemPath, std::string dataPath) {

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

		if(system_status != READY) {
			ROS_INFO("PlanningSystem::runPlanningServer is not READY!");
			return false;
		}

		system_status = PLANNING;
		std_msgs::String statusMsg;
		statusMsg.data = "Planning";
		state_publisher.publish(statusMsg);

		ros::NodeHandle nh("~");

		// parse domain
		environment.parseDomain(domain_path);
	
		// update the environment from the ontology
		environment.update(nh);

		// generate PDDL problem (and problem-specific domain)
		pddl_problem_generator.generatePDDLProblemFile(environment, problem_path);
		ROS_INFO("KCL: (PS) (%s) The problem was generated!", problem_name.c_str());

		// publish problem
		std::ifstream problemIn(problem_path.c_str());
		if(problemIn) {
			std_msgs::String problemMsg;
			problemMsg.data = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
			problem_publisher.publish(problemMsg);
		}

		system_status = READY;
		statusMsg.data = "Ready";
		state_publisher.publish(statusMsg);
		
		return true;
	}


	/*-----------*/
	/* Knowledge */
	/*-----------*/

	/**
	 * listen to and process "/kcl_rosplan/notification" topic.
	 * It is here that the real reasoning takes place.
	 */
	void PlanningSystem::notificationCallBack(const rosplan_knowledge_msgs::Notification::ConstPtr& msg) {
		ROS_INFO("KCL: (PS) (%s) Notification received; plan invalidated; replanning.", problem_name.c_str());
		plan_dispatcher->replan_requested = true;
		
	}

	/**
	 * pushes the filter: the list of items the planner cares about in its plan.
	 */
	void PlanningSystem::publishFilter() {

		// clear the old filter
		rosplan_knowledge_msgs::Filter filterMessage;
		filterMessage.function = rosplan_knowledge_msgs::Filter::CLEAR;
		filter_publisher.publish(filterMessage);

		// push the new filter
		ROS_INFO("KCL: (PS) (%s) Clean and update knowledge filter", problem_name.c_str());
		filterMessage.function = rosplan_knowledge_msgs::Filter::ADD;
		filterMessage.knowledge_items = plan_parser->knowledge_filter;
		filter_publisher.publish(filterMessage);
	}

	/*--------------------------*/
	/* Planning system commands */
	/*--------------------------*/

	void PlanningSystem::commandCallback(const std_msgs::String::ConstPtr& msg) {

		ROS_INFO("KCL: (PS) (%s) Command received: %s", problem_name.c_str(),  msg->data.c_str());

		if(msg->data.substr(0,4) == "plan") {

			// get free action ID
			if(msg->data.length() > 4) {
				size_t aid = atoi(msg->data.substr(5).c_str());
				plan_dispatcher->setCurrentAction(aid);
			}
			ROS_INFO("KCL: (PS) (%s) Processing planning request", problem_name.c_str());
			std_srvs::Empty srv;
			runPlanningServerDefault(srv.request,srv.response);

		} else if(msg->data == "pause") {

			if(system_status == DISPATCHING && !plan_dispatcher->dispatch_paused) {
				ROS_INFO("KCL: (PS) (%s) Pausing dispatch", problem_name.c_str());
				plan_dispatcher->dispatch_paused = true;
				system_status = PAUSED;
				// produce status message
				std_msgs::String statusMsg;
				statusMsg.data = "Paused";
				state_publisher.publish(statusMsg);
			} else if(system_status == PAUSED) {
				ROS_INFO("KCL: (PS) (%s) Resuming dispatch", problem_name.c_str());
				plan_dispatcher->dispatch_paused = false;
				system_status = DISPATCHING;
				// produce status message
				std_msgs::String statusMsg;
				statusMsg.data = "Dispatching";
				state_publisher.publish(statusMsg);
			}

		} else if(msg->data == "cancel") {

			switch(system_status) {
				case PLANNING:
				case DISPATCHING:
				case PAUSED:
					ROS_INFO("KCL: (PS) (%s) Cancelling", problem_name.c_str());
					plan_dispatcher->plan_cancelled = true;
					break;
			}
			if(system_status == PAUSED ) {
				plan_dispatcher->dispatch_paused = false;
				system_status = DISPATCHING;
				// produce status message
				std_msgs::String statusMsg;
				statusMsg.data = "Dispatching";
				state_publisher.publish(statusMsg);
			}
		}
	}

	/*--------------------------*/
	/* Service and Action hooks */
	/*--------------------------*/

	/* planning system service method; loads parameters and calls method below */
	bool PlanningSystem::runPlanningServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		ros::NodeHandle nh;

		data_path = "common/";
		domain_path = "common/domain.pddl";
		problem_path = "common/problem.pddl";
		planner_command = "timeout 60 common/bin/popf -n DOMAIN PROBLEM";
		max_dispatch_attempts = 1;

		// setup environment
		nh.param("/rosplan/domain_path", domain_path, std::string("common/domain.pddl"));
		nh.getParam("/rosplan_planning_system/data_path", data_path);
		nh.getParam("/rosplan_planning_system/problem_path", problem_path);
		nh.getParam("/rosplan_planning_system/planner_command", planner_command);
		nh.getParam("/rosplan_planning_system/max_dispatch_attempts", max_dispatch_attempts);

		// call planning server
		plan_dispatcher->setCurrentAction(0);
		return runPlanningServer(domain_path, problem_path, data_path, planner_command);
	}

	/* planning system action hook */
	void PlanningSystem::runPlanningServerAction(const rosplan_dispatch_msgs::PlanGoalConstPtr& goal) {
		
		// set the starting action ID
		plan_dispatcher->setCurrentAction(goal->start_action_id);
		max_dispatch_attempts = 1;

		// call planning server
		ROS_INFO("KCL: (PS) (%s) Planning Action recieved.", goal->problem_path.c_str());
		if(runPlanningServer(goal->domain_path, goal->problem_path, goal->data_path, goal->planner_command)) {
			plan_server->setSucceeded();
		} else {
			plan_server->setAborted();
		}
	}

	/* planning system service hook with parameters */
	bool PlanningSystem::runPlanningServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res) {
		// call planning server
		max_dispatch_attempts = 1;
		plan_dispatcher->setCurrentAction(0);
		return runPlanningServer(req.domain_path, req.problem_path, req.data_path, req.planner_command);
	}
	
	/*----------------------*/
	/* Planning system loop */
	/*----------------------*/

	/**
	 * planning system service method; prepares planning; main loop.
	 */
	bool PlanningSystem::runPlanningServer(std::string domainPath, std::string problemPath, std::string dataPath, std::string plannerCommand) {

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

		if(system_status != READY) {
			ROS_INFO("PlanningSystem::runPlanningServer is not READY!");
			return false;
		}

		system_status = PLANNING;
		std_msgs::String statusMsg;
		statusMsg.data = "Planning";
		state_publisher.publish(statusMsg);

		ros::NodeHandle nh("~");

		// parse domain
		environment.parseDomain(domain_path);
		
		// dispatch plan
		plan_parser->reset();
		plan_dispatcher->reset();
		plan_dispatcher->environment = environment;

		bool planSucceeded = false;
		mission_start_time = ros::WallTime::now().toSec();
		dispatch_attempts = 0;
		while(!planSucceeded && !plan_dispatcher->plan_cancelled && dispatch_attempts < max_dispatch_attempts) {

			statusMsg.data = "Planning";
			state_publisher.publish(statusMsg);
			system_status = PLANNING;

			// update the environment from the ontology
			environment.update(nh);

			// generate PDDL problem (and problem-specific domain)
			if(generate_problem) {
				pddl_problem_generator.generatePDDLProblemFile(environment, problem_path);
				ROS_INFO("KCL: (PS) (%s) The problem was generated!", problem_name.c_str());
			} else {
				// ROS_INFO("KCL: (PS) Skipping problem generation.");
				rosplan_knowledge_msgs::GenerateProblemService genSrv;
				genSrv.request.problem_path = problem_path;
				genSrv.request.contingent = ("ff" == plannerCommand.substr(0,2));
				if (!generate_problem_client.call(genSrv)) {
					ROS_INFO("KCL: (PS) (%s) The problem was not generated.", problem_name.c_str());
					system_status = READY;
					statusMsg.data = "Ready";
					state_publisher.publish(statusMsg);
					return false;
				}
				ROS_INFO("KCL: (PS) (%s) The problem was generated!", problem_name.c_str());
			}

			// publish problem
			std::ifstream problemIn(problem_path.c_str());
			if(problemIn) {
				std_msgs::String problemMsg;
				problemMsg.data = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
				problem_publisher.publish(problemMsg);
			}

			// run planner; generate a plan
			if(!runPlanner()) {
				system_status = READY;
				statusMsg.data = "Ready";
				state_publisher.publish(statusMsg);
				return false;
			}

			// publish plan
			rosplan_dispatch_msgs::CompletePlan planMsg;
			planMsg.plan = plan_parser->action_list;
			plan_publisher.publish(planMsg);

			// re-parse the (problem-specific) domain
			environment.parseDomain(domain_path);

			// dispatch plan
			system_status = DISPATCHING;
			statusMsg.data = "Dispatching";
			state_publisher.publish(statusMsg);
			plan_start_time = ros::WallTime::now().toSec();
			planSucceeded = plan_dispatcher->dispatchPlan(plan_parser->action_list, mission_start_time, plan_start_time);
			
			if (!planSucceeded) {
				ROS_INFO("KCL: (PS) (%s) The plan failed!", problem_name.c_str());
			} else {
				ROS_INFO("KCL: (PS) (%s) Dispatch completed.", problem_name.c_str());
			}
		}
		ROS_INFO("KCL: (PS) (%s) Planning System Finished", problem_name.c_str());

		system_status = READY;
		statusMsg.data = "Ready";
		state_publisher.publish(statusMsg);
		
		return planSucceeded;
	}

	/*------------------*/
	/* Plan and process */
	/*------------------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 * This method is popf-specific.
	 */
	bool PlanningSystem::runPlanner() {

		// save previous plan
		planning_attempts++;
		if(plan_parser->action_list.size() > 0) {
			std::vector<rosplan_dispatch_msgs::ActionDispatch> oldplan(plan_parser->action_list);
			plan_list.push_back(oldplan);
		}

		// run the planner
		std::string str = planner_command;
		std::size_t dit = str.find("DOMAIN");
		if(dit!=std::string::npos) str.replace(dit,6,domain_path);
		std::size_t pit = str.find("PROBLEM");
		if(pit!=std::string::npos) str.replace(pit,7,problem_path);
		
		std::string commandString = str + " > " + data_path + "plan.pddl";
		ROS_INFO("KCL: (PS) (%s) Running: %s", problem_name.c_str(),  commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("KCL: (PS) (%s) Planning complete", problem_name.c_str());

		// check the planner solved the problem (TODO standardised PDDL2.1 output)
		std::ifstream planfile;
		planfile.open((data_path + "plan.pddl").c_str());
		std::string line;
		bool solved = false;
		
		while(!planfile.eof() && !solved) {
			getline(planfile, line);
			if (line.find("; Plan found", 0) != std::string::npos)
				solved = true;
			if (line.find("ff: found legal plan as follows", 0) != std::string::npos)
				solved = true;
		}
		if(!solved) {
				planfile.close();
				ROS_INFO("KCL: (PS) (%s) Plan was unsolvable! Try again?", problem_name.c_str());
				return false;
		}

		// save file
		std::stringstream ss;
		ss << planning_attempts;
		std::ifstream source;
		std::ofstream dest;
		source.open((data_path + "plan.pddl").c_str());
		dest.open((data_path + "plan_" + ss.str()).c_str());
		dest << source.rdbuf();
		source.close();
		dest.close();

		// Convert plan into message list for dispatch
		plan_parser->preparePlan(data_path, environment, plan_dispatcher->getCurrentAction());

		// publish the filter to the knowledge base
		publishFilter();

		return true;
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		srand (static_cast <unsigned> (time(0)));

		ros::init(argc,argv,"rosplan_planning_system");
		ros::NodeHandle nh;

		KCL_rosplan::PlanningSystem planningSystem(nh);

		ros::Subscriber feedback_sub = nh.subscribe("/kcl_rosplan/action_feedback", 10, &KCL_rosplan::PlanDispatcher::feedbackCallback, planningSystem.plan_dispatcher);
		ros::Subscriber command_sub = nh.subscribe("/kcl_rosplan/planning_commands", 10, &KCL_rosplan::PlanningSystem::commandCallback, &planningSystem);

		// publishing "/kcl_rosplan/filter"; listening "/kcl_rosplan/notification"
		planningSystem.filter_publisher = nh.advertise<rosplan_knowledge_msgs::Filter>("/kcl_rosplan/planning_filter", 10, true);
		ros::Subscriber notification_sub = nh.subscribe("/kcl_rosplan/notification", 10, &KCL_rosplan::PlanningSystem::notificationCallBack, &planningSystem);

		// start a problem generation service
		bool genProb = true;
		nh.getParam("/rosplan_planning_system/generate_default_problem", genProb);
		planningSystem.generate_problem = genProb;

		if(genProb) {
			ROS_INFO("KCL: (PS) Using the standard PDDL problem generator.");
		} else {
			ROS_INFO("KCL: (PS) Using an alternative PDDL problem generator.");
		}
		
		// start the planning services
		ros::ServiceServer service1 = nh.advertiseService("/kcl_rosplan/planning_server", &KCL_rosplan::PlanningSystem::runPlanningServerDefault, &planningSystem);
		ros::ServiceServer service2 = nh.advertiseService("/kcl_rosplan/planning_server_params", &KCL_rosplan::PlanningSystem::runPlanningServerParams, &planningSystem);
		ros::ServiceServer service4 = nh.advertiseService("/kcl_rosplan/problem_server_params", &KCL_rosplan::PlanningSystem::runProblemServerParams, &planningSystem);

		std_msgs::String statusMsg;
		statusMsg.data = "Ready";
		planningSystem.state_publisher.publish(statusMsg);

		ROS_INFO("KCL: (PS) Ready to receive");
		ros::spin();

		return 0;
	}
