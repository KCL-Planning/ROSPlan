#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_dispatch_msgs/PlanningService.h"
#include "rosplan_dispatch_msgs/PlanAction.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "rosplan_knowledge_msgs/GenerateProblemService.h"

#include "PlanningEnvironment.h"
#include "PDDLProblemGenerator.h"

#include "PlanParser.h"
#include "POPFPlanParser.h"
#include "POPFEsterelPlanParser.h"
#include "CFFPlanParser.h"

#include "PlanDispatcher.h"
#include "SimplePlanDispatcher.h"
#include "EsterelPlanDispatcher.h"

#ifndef KCL_planning_system
#define KCL_planning_system

/**
 * This file includes the main loop of the planning node.
 */
namespace KCL_rosplan {

	/* status */
	enum SystemStatus { READY, PLANNING, DISPATCHING, PAUSED };

	class PlanningSystem
	{
	private:

		ros::NodeHandle nh_;

		/* simulation information */
		bool use_plan_visualisation;

		/* runs external commands */
		std::string runCommand(std::string cmd);

		/* paths */
		std::string planner_command;
		std::string domain_path;
		std::string problem_path;
		std::string problem_name;
		std::string data_path;

		/* planning */
		actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>* plan_server;
		double mission_start_time;
		double plan_start_time;
		bool runPlanner();

		/* plan list (for remembering previous plans) */
		std::vector< std::vector<rosplan_dispatch_msgs::ActionDispatch> > plan_list;
		size_t planning_attempts;
		size_t dispatch_attempts;
		int max_dispatch_attempts;

	public:
		PlanningSystem(ros::NodeHandle& nh);
		virtual ~PlanningSystem();
		
		SystemStatus system_status;

		void commandCallback(const std_msgs::String::ConstPtr& msg);
		bool runPlanningServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool runPlanningServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res);
		void runPlanningServerAction(const rosplan_dispatch_msgs::PlanGoalConstPtr& goal);
		bool runPlanningServer(std::string domainPath, std::string problemPath, std::string dataPath, std::string plannerCommand);


		/* knowledge */
		PlanningEnvironment environment;
		bool generatePlanningProblem(ros::NodeHandle nh, std::string &problemPath);
		void notificationCallBack(const rosplan_knowledge_msgs::Notification::ConstPtr& msg);

		/* problem generation */
		bool runProblemServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res);
		bool runProblemServer(std::string domainPath, std::string problemPath, std::string dataPath);

		/* planning */
		ros::ServiceClient generate_problem_client;
		PDDLProblemGenerator pddl_problem_generator;
		PlanParser* plan_parser;
		bool generate_problem;
		bool generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);
		void publishFilter();
	
		/* dispatch class */
		PlanDispatcher* plan_dispatcher;

		/* ROS interface */
		ros::Publisher filter_publisher;
		ros::Publisher problem_publisher;
		ros::Publisher plan_publisher;
		ros::Publisher state_publisher;
	};

} // close namespace

#endif
