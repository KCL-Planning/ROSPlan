#include "rosplan_planning_system/PlanningSystem.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>

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
		ROS_INFO("KCL: Notification received; plan invalidated; replanning.");
		// replanRequested = true;
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
		ROS_INFO("KCL: Clean and update knowledge filter");
		filterMessage.function = rosplan_knowledge_msgs::Filter::ADD;
		filterMessage.knowledge_items = knowledge_filter;
		filter_publisher.publish(filterMessage);
	}

	/*----------*/
	/* Planning */
	/*----------*/

	/**
	 * passes the problem to the Planner; the plan to post-processing.
	 * This method is popf-specific.
	 */
	bool PlanningSystem::runPlanner()
	{
		// save previous plan
		planningAttempts = planningAttempts + 1;
		if(action_list.size() > 0) {
			std::vector<rosplan_dispatch_msgs::ActionDispatch> oldplan(action_list);
			plan_list.push_back(oldplan);
		}

		// run the planner
		std::string commandString = planner_command + " "
			 + domain_path + " " + problem_path + " > "
			 + data_path + "plan.pddl";

		ROS_INFO("KCL: Running: %s", commandString.c_str());
		std::string plan = runCommand(commandString.c_str());
		ROS_INFO("KCL: Planning complete");

		// check the planner solved the problem (TODO standardised PDDL2.1 output)
		std::ifstream planfile;
		planfile.open((data_path + "plan.pddl").c_str());
		std::string line;
		while(!planfile.eof()) {
			getline(planfile, line);
			if (line.substr(0,22).compare(";; Problem unsolvable!")==0) { 
				planfile.close();
				ROS_INFO("Plan was unsolvable. Trying again.");
				return false;
			}
		}
		planfile.close();
		
		ROS_INFO("KCL: Post processing plan");

		// trim the end of any existing plan
		free_action_ID = plan_dispatcher.getCurrentAction();

		// Convert plan into message list for dispatch
		// preparePlan(data_path);

		// publish the filter to the knowledge base
		publishFilter();

		return true;
	}

	/**
	 * requests objects; generates the PDDL problem.
	 */
	bool PlanningSystem::generatePlanningProblem(ros::NodeHandle nh, std::string &problemPath)
	{
		// update the environment from the ontology
		ROS_INFO("KCL: Fetching objects");
		environment.update(nh);

		// generate PDDL problem
		// generatePDDLProblemFile(problemPath);

		return true;
	}

	/**
	 * sets up the ROS node; prepares planning; main loop.
	 */
	bool PlanningSystem::runPlanningServer(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
	{
		ros::NodeHandle nh("~");

		// setup environment
		ROS_INFO("KCL: Setup domain");
		nh.param("data_path", data_path, std::string("common/"));
		nh.param("domain_path", domain_path, std::string("common/domain.pddl"));
		nh.param("problem_path", problem_path, std::string("common/problem.pddl"));
		nh.param("planner_command", planner_command, std::string("timeout 10 common/bin/popf -n"));
		environment.parseDomain(domain_path);
	
		// dispatch plan
		bool planSucceeded = false;
		if(!planSucceeded) {

			// generate PDDL problem and run planner
			ROS_INFO("KCL: Generate problem");
			// generatePlanningProblem(nh, dataPath);
			// runPlanner(dataPath, domainPath, problemPath);

			// dispatch plan
			ROS_INFO("KCL: Dispatch plan");
			plan_dispatcher.dispatchPlan(action_list, mission_start_time, plan_start_time);
		}
		ROS_INFO("KCL: Planning System Finished");
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

		// publishing "action_dispatch"; listening "action_feedback"
		planningSystem.plan_dispatcher.action_publisher = nh.advertise<rosplan_dispatch_msgs::ActionDispatch>("/kcl_rosplan/action_dispatch", 1000, true);
		nh.subscribe("/kcl_rosplan/action_feedback", 100, &KCL_rosplan::PlanDispatcher::feedbackCallback, &planningSystem.plan_dispatcher);

		// publishing "/kcl_rosplan/filter"; listening "/kcl_rosplan/notification"
		planningSystem.filter_publisher = nh.advertise<rosplan_knowledge_msgs::Filter>("/kcl_rosplan/filter", 10, true);
		nh.subscribe("/kcl_rosplan/notification", 100, &KCL_rosplan::PlanningSystem::notificationCallBack, &planningSystem);

		// start te planning service
		ros::ServiceServer service = nh.advertiseService("/kcl_rosplan/planning_server", &KCL_rosplan::PlanningSystem::runPlanningServer, &planningSystem);
		ROS_INFO("KCL: Ready to receive");
		ros::spin();

		return 0;
	}
