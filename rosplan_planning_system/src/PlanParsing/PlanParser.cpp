#include "rosplan_planning_system/PlanParsing/PlanParser.h"

namespace KCL_rosplan {

	/*----------------------*/
	/* Planner subscription */
	/*----------------------*/

	void PlanParser::plannerCallback(const std_msgs::String& plannerOutput) {
        
		ROS_INFO("KCL: (%s) Planner output received.", ros::this_node::getName().c_str());        
		ROS_INFO("KCL: (%s) Is plan empty?: %d", ros::this_node::getName().c_str(), plannerOutput.data.size() == 0);
        
		planner_output_received = true;
		planner_output_time = ros::Time::now().toSec();
		planner_output = plannerOutput.data;
	}

	/*-------------------*/
	/* Parsing interface */
	/*-------------------*/

	/**
	 * plan parsing service method (1) 
	 * loads plan from file
	 */
	bool PlanParser::parsePlanFromFile(rosplan_dispatch_msgs::ParsingService::Request &req, rosplan_dispatch_msgs::ParsingService::Response &res) {

		ROS_INFO("KCL: (%s) Parsing plan from file.", ros::this_node::getName().c_str());

		reset();

		// load plan from file
		std::stringstream ss;
		std::ifstream source;
		source.open((req.plan_path).c_str());
		ss << source.rdbuf();
		planner_output = ss.str();
		source.close();

		planner_output_received = true;

		preparePlan();
		publishPlan();

		planner_output_received = false;

		res.plan_parsed = true;
		return true;
	}

	/**
	 * plan parsing service method (2) 
	 * parse last plan read
	 */
	bool PlanParser::parsePlan(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {

		if(planner_output_received) {
			ROS_INFO("KCL: (%s) Parsing planner output.", ros::this_node::getName().c_str());   
			reset();
			preparePlan();
			publishPlan();
			planner_output_received = false;
		} else {
			ROS_INFO("KCL: (%s) No new planner output received; nothing to parse.", ros::this_node::getName().c_str());
		}
		return true;
	}
} // close namespace
