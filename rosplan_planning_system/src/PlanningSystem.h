#ifndef KCL_planning_system
#define KCL_planning_system

#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "PlanDispatcher.h"
#include "Utilities.h"

namespace KCL_rosplan {

	class PlanningSystem
	{
	private:

		/* Runs external commands */
		std::string runCommand(std::string cmd);
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

		void runPlanningServer();

		/* knowledge */
		bool generatePlanningProblem(ros::NodeHandle nh, std::string &problemPath);
		void notificationCallBack(const rosplan_knowledge_msgs::Notification::ConstPtr& msg);

		/* planning */
		bool runPlanner();
		void publishFilter();
	
		/* dispatch */
		PlanDispatcher planDispatcher;

		/* ROS interface */
		ros::Publisher filterPublisher;
	};

} // close namespace

#endif
