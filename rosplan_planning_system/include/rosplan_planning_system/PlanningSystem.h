/**
 * This file includes the main loop of the planning node.
 * TODO document
 */
#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "PlanningEnvironment.h"
#include "PDDLProblemGenerator.h"
#include "PlanParser.h"
#include "POPFPlanParser.h"
#include "PlanDispatcher.h"
#include "SimplePlanDispatcher.h"
#include "EsterelPlanDispatcher.h"

#ifndef KCL_planning_system
#define KCL_planning_system

namespace KCL_rosplan {

	/* status */
	enum SystemStatus { READY, PLANNING, DISPATCHING, PAUSED };

	class PlanningSystem
	{
	private:

		/* simulation information */
		bool use_plan_visualisation;

		/* runs external commands */
		std::string runCommand(std::string cmd);

		/* paths */
		std::string planner_command;
		std::string domain_path;
		std::string problem_path;
		std::string data_path;

		/* planning */
		double mission_start_time;
		double plan_start_time;
		bool runPlanner();

		/* plan list (for remembering previous plans) */
		std::vector< std::vector<rosplan_dispatch_msgs::ActionDispatch> > plan_list;
		size_t planning_attempts;

	public:

		SystemStatus system_status;

		void commandCallback(const std_msgs::String::ConstPtr& msg);
		bool runPlanningServer(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

		/* knowledge */
		PlanningEnvironment environment;
		bool generatePlanningProblem(ros::NodeHandle nh, std::string &problemPath);
		void notificationCallBack(const rosplan_knowledge_msgs::Notification::ConstPtr& msg);

		/* planning */
		PDDLProblemGenerator pddl_problem_generator;
		POPFPlanParser plan_parser;
		void publishFilter();
	
		/* dispatch class */
		PlanDispatcher* plan_dispatcher;

		/* ROS interface */
		ros::Publisher filter_publisher;
		ros::Publisher plan_publisher;
		ros::Publisher state_publisher;
	};

} // close namespace

#endif
