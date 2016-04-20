#include "rosplan_planning_system/PlanningSystem.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <ros/package.h>

namespace KCL_rosplan {

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

	/*-----------*/
	/* Knowledge */
	/*-----------*/

	/**
	 * listen to and process "/kcl_rosplan/notification" topic.
	 * It is here that the real reasoning takes place.
	 */
	void PlanningSystem::notificationCallBack(const rosplan_knowledge_msgs::Notification::ConstPtr& msg) {
		ROS_INFO("KCL: (PS) Notification received; plan invalidated; replanning.");
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
		ROS_INFO("KCL: (PS) Clean and update knowledge filter");
		filterMessage.function = rosplan_knowledge_msgs::Filter::ADD;
		filterMessage.knowledge_items = plan_parser.knowledge_filter;
		filter_publisher.publish(filterMessage);
	}

	/*----------------------*/
	/* Planning system loop */
	/*----------------------*/

	void PlanningSystem::commandCallback(const std_msgs::String::ConstPtr& msg) {
		ROS_INFO("KCL: (PS) Command received: %s", msg->data.c_str());
		if(msg->data == "plan") {
			if(system_status == READY) {
				ROS_INFO("KCL: (PS) Processing planning request");
				std_srvs::Empty srv;
				runPlanningServer(srv.request,srv.response);
			}
		} else if(msg->data == "pause") {
			if(system_status == DISPATCHING && !plan_dispatcher->dispatch_paused) {
				ROS_INFO("KCL: (PS) Pausing dispatch");
				plan_dispatcher->dispatch_paused = true;
				system_status = PAUSED;
				// produce status message
				std_msgs::String statusMsg;
				statusMsg.data = "Paused";
				state_publisher.publish(statusMsg);
			} else if(system_status == PAUSED) {
				ROS_INFO("KCL: (PS) Resuming dispatch");
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
					ROS_INFO("KCL: (PS) Cancelling");
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

	/**
	 * planning system service method; prepares planning; main loop.
	 */
	bool PlanningSystem::runPlanningServer(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		if(system_status != READY) return false;

		system_status = PLANNING;
		std_msgs::String statusMsg;
		statusMsg.data = "Planning";
		state_publisher.publish(statusMsg);

		ros::NodeHandle nh("~");

		// setup environment
		std::string pkg_path = ros::package::getPath("rosplan_planning_system");

		nh.param("~domain_path", domain_path, pkg_path + std::string("/common/domain.pddl"));
		nh.param("~data_path", data_path, pkg_path + std::string("/common/"));
		nh.param("~problem_path", problem_path, pkg_path + std::string("/common/problem.pddl"));
		nh.param("~planner_command", planner_command, 
			  std::string("timeout 10 ") 
			+ pkg_path
			+ std::string("/common/bin/popf -n"));

		environment.parseDomain(domain_path);

		// dispatch plan
		plan_parser.reset();
		plan_dispatcher->reset();
		plan_dispatcher->environment = environment;

		bool planSucceeded = false;
		mission_start_time = ros::WallTime::now().toSec();
		while(!planSucceeded && !plan_dispatcher->plan_cancelled) {

			statusMsg.data = "Planning";
			state_publisher.publish(statusMsg);
			system_status = PLANNING;

			// update the environment from the ontology
			environment.update(nh);

			// generate PDDL problem
			pddl_problem_generator.generatePDDLProblemFile(environment, problem_path);

			// run planner; generate a plan
			runPlanner();

			// publish plan
			rosplan_dispatch_msgs::CompletePlan planMsg;
			planMsg.plan = plan_parser.action_list;
			plan_publisher.publish(planMsg);

			// dispatch plan
			system_status = DISPATCHING;
			statusMsg.data = "Dispatching";
			state_publisher.publish(statusMsg);
			plan_start_time = ros::WallTime::now().toSec();
			planSucceeded = plan_dispatcher->dispatchPlan(plan_parser.action_list, mission_start_time, plan_start_time);
		}
		ROS_INFO("KCL: (PS) Planning System Finished");

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
		if(plan_parser.action_list.size() > 0) {
			std::vector<rosplan_dispatch_msgs::ActionDispatch> oldplan(plan_parser.action_list);
			plan_list.push_back(oldplan);
		}

		// run the planner (TODO standardised PDDL2.1 input)
		std::string commandString = planner_command
				+ " " + domain_path + " " + problem_path
				 + " > " + data_path + "plan.pddl";
		ROS_INFO("KCL: (PS) Running: %s", commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("KCL: (PS) Planning complete");

		// check the planner solved the problem (TODO standardised PDDL2.1 output)
		std::ifstream planfile;
		planfile.open((data_path + "plan.pddl").c_str());
		std::string line;
		bool solved = false;
		while(!planfile.eof() && !solved) {
			getline(planfile, line);
			if (line.find("; Time", 0) != std::string::npos)
				solved = true;
		}
		if(!solved) {
				planfile.close();
				ROS_INFO("KCL: (PS) Plan was unsolvable! Try again?");
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
		plan_parser.preparePlan(data_path, environment, plan_dispatcher->getCurrentAction());

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

		KCL_rosplan::PlanningSystem planningSystem;

		// plan dispatcher
		KCL_rosplan::SimplePlanDispatcher spd;
		planningSystem.plan_dispatcher = &spd;

		// publishing system_state
		planningSystem.state_publisher = nh.advertise<std_msgs::String>("/kcl_rosplan/system_state", 5, true);

		// publishing "action_dispatch", "action_feedback", "plan"; listening "action_feedback"
		planningSystem.plan_publisher = nh.advertise<rosplan_dispatch_msgs::CompletePlan>("/kcl_rosplan/plan", 5, true);
		planningSystem.plan_dispatcher->action_publisher = nh.advertise<rosplan_dispatch_msgs::ActionDispatch>("/kcl_rosplan/action_dispatch", 1000, true);
		planningSystem.plan_dispatcher->action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 5, true);
		ros::Subscriber feedback_sub = nh.subscribe("/kcl_rosplan/action_feedback", 10, &KCL_rosplan::PlanDispatcher::feedbackCallback, planningSystem.plan_dispatcher);
		ros::Subscriber command_sub = nh.subscribe("/kcl_rosplan/planning_commands", 10, &KCL_rosplan::PlanningSystem::commandCallback, &planningSystem);

		// publishing "/kcl_rosplan/filter"; listening "/kcl_rosplan/notification"
		planningSystem.filter_publisher = nh.advertise<rosplan_knowledge_msgs::Filter>("/kcl_rosplan/planning_filter", 10, true);
		ros::Subscriber notification_sub = nh.subscribe("/kcl_rosplan/notification", 10, &KCL_rosplan::PlanningSystem::notificationCallBack, &planningSystem);

		// start the planning service
		ros::ServiceServer service = nh.advertiseService("/kcl_rosplan/planning_server", &KCL_rosplan::PlanningSystem::runPlanningServer, &planningSystem);
		planningSystem.system_status = KCL_rosplan::READY;
		std_msgs::String statusMsg;
		statusMsg.data = "Ready";
		planningSystem.state_publisher.publish(statusMsg);
		ROS_INFO("KCL: (PS) Ready to receive");
		ros::spin();

		return 0;
	}
