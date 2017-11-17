#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "rosplan_dispatch_msgs/CompletePlan.h"
#include "rosplan_dispatch_msgs/PlanningService.h"
#include "rosplan_dispatch_msgs/PlanAction.h"
#include "rosplan_knowledge_msgs/GenerateProblemService.h"

#include "PlanningEnvironment.h"
#include "PDDLProblemGenerator.h"

#include "PlanParser.h"
#include "POPFPlanParser.h"
#include "POPFEsterelPlanParser.h"
#include "CFFPlanParser.h"
#include "CLGPlanParser.h"

#ifndef KCL_planner_interface
#define KCL_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class PlannerInterface
	{
	private:

		ros::NodeHandle nh_;

		/* runs external commands */
		std::string runCommand(std::string cmd);

		/* params */
		std::string planner_command;
		std::string domain_path;
		std::string problem_path;
		std::string problem_name;
		std::string data_path;

		/* planning */
		actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>* plan_server;
		bool runPlanner();

	public:

		PlannerInterface(ros::NodeHandle& nh);
		virtual ~PlannerInterface();

		bool runPlanningServer(std::string domainPath, std::string problemPath, std::string dataPath, std::string plannerCommand);
		bool runPlanningServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool runPlanningServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res);
		void runPlanningServerAction(const rosplan_dispatch_msgs::PlanGoalConstPtr& goal);

		/* knowledge */
		PlanningEnvironment environment;
		bool generatePlanningProblem(ros::NodeHandle nh, std::string &problemPath);

		/* problem generation */
		bool runProblemServer(std::string domainPath, std::string problemPath, std::string dataPath);
		bool runProblemServerParams(rosplan_dispatch_msgs::PlanningService::Request &req, rosplan_dispatch_msgs::PlanningService::Response &res);

		/* planning */
		PDDLProblemGenerator pddl_problem_generator;
		PlanParser* plan_parser;

		/* ROS interface */
		ros::Publisher problem_publisher;
		ros::Publisher plan_publisher;
	};

} // close namespace

#endif
