/**
 * This file includes the main loop of the planning node.
 * TODO document
 */
#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "std_srvs/Empty.h"
#include "PlanningEnvironment.h"
#include "PlanDispatcher.h"

#ifndef KCL_planning_system
#define KCL_planning_system

namespace KCL_rosplan {

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
		size_t free_action_ID;
		double mission_start_time;
		double plan_start_time;

		/* plan list (for remembering previous plans) */
		std::vector< std::vector<rosplan_dispatch_msgs::ActionDispatch> > plan_list;
		std::vector<rosplan_dispatch_msgs::ActionDispatch> action_list;
		size_t planningAttempts;

	public:

		bool runPlanningServer(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

		/* knowledge */
		PlanningEnvironment environment;
		bool generatePlanningProblem(ros::NodeHandle nh, std::string &problemPath);
		void notificationCallBack(const rosplan_knowledge_msgs::Notification::ConstPtr& msg);

		/* planning */
		std::vector<std::string> filter_objects;
		std::vector<std::vector<std::string> > filter_attributes;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> knowledge_filter;
		bool runPlanner();
		void publishFilter();
	
		/* dispatch */
		PlanDispatcher plan_dispatcher;

		/* ROS interface */
		ros::Publisher filter_publisher;
	};

} // close namespace

#endif
