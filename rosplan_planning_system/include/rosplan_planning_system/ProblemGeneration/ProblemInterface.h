#include "ros/ros.h"

#include "ProblemGeneratorFactory.h"

#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include "rosplan_dispatch_msgs/ProblemService.h"

#ifndef KCL_problem_interface
#define KCL_problem_interface

/**
 * This file contains an interface to the problem.
 */
namespace KCL_rosplan {

	class ProblemInterface
	{
	private:

		ros::NodeHandle* node_handle;

		/* params */
		std::string problem_path;
		std::string problem_name;

		ProblemGeneratorPtr problem_generator;

	public:

		ProblemInterface(ros::NodeHandle& nh);
		virtual ~ProblemInterface();

		bool runProblemServer(std::string problemPath);
		bool runProblemServerDefault(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool runProblemServerParams(rosplan_dispatch_msgs::ProblemService::Request &req, rosplan_dispatch_msgs::ProblemService::Response &res);

		/* ROS interface */
		ros::Publisher problem_publisher;
	};

} // close namespace

#endif
