#ifndef KCL_planner_interface
#define KCL_planner_interface

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "rosplan_dispatch_msgs/PlanningService.h"
#include "rosplan_dispatch_msgs/PlanAction.h"

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class PlannerInterface
	{
	private:
	protected:

		ros::NodeHandle* node_handle;

		actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>* plan_server;

		/* params */
		bool use_problem_topic;
		std::string planner_command;
		std::string domain_path;
		std::string problem_path;
		std::string problem_name;
		std::string data_path;

		/* planner outputs */
		std::string planner_output;

		/* problem subscription */
		std::string problem_instance;
		bool problem_instance_received;
		double problem_instance_time;

		/* planning */
		virtual bool runPlanner() =0;

	public:

		void problemCallback(const std_msgs::String& problemInstance);

		bool runPlanningServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool runPlanningServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res);
		void runPlanningServerAction(const rosplan_dispatch_msgs::PlanGoalConstPtr& goal);
		bool runPlanningServer(std::string domainPath, std::string problemPath, std::string dataPath, std::string plannerCommand, bool useProblemTopic);

		/* ROS interface */
		ros::Publisher plan_publisher;
	};

} // close namespace

#endif
