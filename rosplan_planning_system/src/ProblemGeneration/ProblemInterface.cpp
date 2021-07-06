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

		// connecting to KB
		std::string kb = "knowledge_base";
		node_handle->getParam("knowledge_base", kb);

		// Check problem fle extension to select a problem generator
		std::string planning_lang;
		node_handle->param<std::string>("planning_language", planning_lang, "");
		std::transform(planning_lang.begin(), planning_lang.end(), planning_lang.begin(), ::tolower); // Convert to lowercase
		node_handle->param<std::string>("problem_path", problem_path, "");

		if (problem_path.empty() and planning_lang.empty()) {
			ROS_ERROR("KCL: (%s) Parameters problem_path and planning_language are both undefined. At least one of them must be defined.",
					  ros::this_node::getName().c_str());
			ros::shutdown();
		}

		KCL_rosplan::ProblemGeneratorFactory::ProbGen pg_type;

		if (not planning_lang.empty()) {
			if (planning_lang == "pddl" or planning_lang == "ppddl") pg_type = KCL_rosplan::ProblemGeneratorFactory::PDDL;
			else if (planning_lang == "rddl") pg_type = KCL_rosplan::ProblemGeneratorFactory::RDDL;
			else if (planning_lang == "chimp") pg_type = KCL_rosplan::ProblemGeneratorFactory::CHIMP;
			else {
				ROS_WARN("KCL: (%s) Unexpected planning language %s. Please specify the planning language as either \"PDDL\" or \"RDDL\" or \"CHIMP\".",
						  ros::this_node::getName().c_str(), planning_lang.c_str());
			}
		} // If problem path is specified, it'll overwrite the setting from the planning_lang parameter.
        if (not problem_path.empty()) {
            std::string extension = (problem_path.size() > 5) ? problem_path.substr(problem_path.find_last_of('.')) : "";
            if (extension == ".pddl" or extension == ".ppddl") pg_type = KCL_rosplan::ProblemGeneratorFactory::PDDL;
            else if (extension == ".rddl") pg_type = KCL_rosplan::ProblemGeneratorFactory::RDDL;
			else if (extension == ".pdl") pg_type = KCL_rosplan::ProblemGeneratorFactory::CHIMP;
            else {
                ROS_ERROR("KCL: (%s) Unexpected problem file extension %s. Please provide a problem file written in PDDL (.pddl extension) or RDDL (.rddl extension) or CHIMP (.pdl extension) format.",
                          ros::this_node::getName().c_str(), extension.c_str());
                ros::shutdown();
            }
        }
        else {
            ROS_ERROR("KCL: (%s) Neither problem file nor planning language was specified! Please specify the parameter to know the planning language.",
                      ros::this_node::getName().c_str());
            ros::shutdown();
		}
		problem_generator = KCL_rosplan::ProblemGeneratorFactory::createProblemGenerator(pg_type, kb);

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
		ROS_INFO("problem server params request received !!");

		// call problem server
		bool success = runProblemServer(req.problem_path);
		if(req.problem_string_response) {
			std::ifstream problemIn(req.problem_path.c_str());
			if(problemIn) res.problem_string = std::string(std::istreambuf_iterator<char>(problemIn), std::istreambuf_iterator<char>());
		}
		res.problem_generated = success;
		return true;
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
		problem_generator->generateProblemFile(problem_path);
		ROS_INFO("KCL: (%s) (%s) The problem was generated.", ros::this_node::getName().c_str(), problem_name.c_str());

		// publish problem
		std::ifstream problemIn(problem_path.c_str());
		if(problemIn) {
			ROS_DEBUG("KCL: (%s) (%s) Publish problem.", ros::this_node::getName().c_str(), problem_name.c_str());
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
