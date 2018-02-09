#include "rosplan_planning_system/ProblemGeneration/ProblemInterface.h"
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

	ProblemInterface::ProblemInterface(ros::NodeHandle& nh)
	{
		node_handle = &nh;

		// publishing "problem"
		std::string problem_instance = "problem_instance";
		node_handle->getParam("problem_topic", problem_instance);
		problem_publisher = node_handle->advertise<std_msgs::String>(problem_instance, 1, true);
	}
	
	ProblemInterface::~ProblemInterface()
	{

	}

	/*--------------------*/
	/* Problem interface */
	/*--------------------*/

	/**
	 * problem generation service method (1) 
	 * loads parameters from param server
	 */
	bool ProblemInterface::runProblemServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		// defaults
		problem_path = "common/problem.pddl";

		// load params
		node_handle->getParam("problem_path", problem_path);

		// call problem server
		return runProblemServer(problem_path);
	}

	/**
	 * problem generation service method (2) 
	 * loads parameters from service request
	 */
	bool ProblemInterface::runProblemServerParams(rosplan_dispatch_msgs::ProblemService::Request &req, rosplan_dispatch_msgs::ProblemService::Response &res) {
		// call problem server
		bool success = runProblemServer(req.problem_path);
		if(req.problem_string_response) {
			std::ifstream problemIn(req.problem_path.c_str());
			if(problemIn) res.problem_string = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
		}
		return success;
	}
	
	/**
	 * planning system; prepares planning; calls planner; parses plan.
	 */
	bool ProblemInterface::runProblemServer(std::string problemPath) {

		ros::NodeHandle nh("~");

		// save parameter
		problem_path = problemPath;
		
		// set problem name for ROS_INFO
		std::size_t lastDivide = problem_path.find_last_of("/\\");
		if(lastDivide != std::string::npos) {
			problem_name = problem_path.substr(lastDivide+1);
		} else {
			problem_name = problem_path;
		}

		ROS_INFO("KCL: (%s) (%s) Generating problem file.", ros::this_node::getName().c_str(), problem_name.c_str());
		pddl_problem_generator.generatePDDLProblemFile(problem_path);
		ROS_INFO("KCL: (%s) (%s) The problem was generated.", ros::this_node::getName().c_str(), problem_name.c_str());

		// publish problem
		std::ifstream problemIn(problem_path.c_str());
		if(problemIn) {
			std_msgs::String problemMsg;
			problemMsg.data = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
			problem_publisher.publish(problemMsg);
		}

		return true;
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		srand (static_cast <unsigned> (time(0)));

		ros::init(argc,argv,"rosplan_problem_interface");
		ros::NodeHandle nh("~");

		KCL_rosplan::ProblemInterface ProblemInterface(nh);
		
		// start the planning services
		ros::ServiceServer service1 = nh.advertiseService("problem_generation_server", &KCL_rosplan::ProblemInterface::runProblemServerDefault, &ProblemInterface);
		ros::ServiceServer service2 = nh.advertiseService("problem_generation_server_params", &KCL_rosplan::ProblemInterface::runProblemServerParams, &ProblemInterface);

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
