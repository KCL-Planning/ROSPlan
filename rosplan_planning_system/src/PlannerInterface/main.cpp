#include <chrono>
#include <thread>
#include <iostream>
#include <string>
#include <fstream>

#include "rosplan_planning_system/PlannerInterface/PlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/UPMPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/POPFPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/FFPlannerInterface.h"
#include <rosplan_planning_system/PlannerInterface/OnlinePlannerInterface.h>
#include "rosplan_planning_system/PlannerInterface/TFDPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/CHIMPPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/SMTPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/RDDLSIMPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/PPRPPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/PANDAPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/LPGPlannerInterface.h"
#include "rosplan_planning_system/PlannerInterface/FDPlannerInterface.h"

template <typename T>
void runPlanner(T & pi, ros::NodeHandle & nh){
	// subscribe to problem instance
	std::string problemTopic = "problem_instance";
	nh.getParam("problem_topic", problemTopic);
	ros::Subscriber problem_sub = nh.subscribe(problemTopic, 1, &KCL_rosplan::PlannerInterface::problemCallback, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

	// start the planning services
	ros::ServiceServer service1 = nh.advertiseService("planning_server", &KCL_rosplan::PlannerInterface::runPlanningServerDefault, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));
	ros::ServiceServer service2 = nh.advertiseService("planning_server_params", &KCL_rosplan::PlannerInterface::runPlanningServerParams, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

	ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
	ros::spin();
}

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

	srand (static_cast <unsigned> (time(0)));

	ros::init(argc,argv,"rosplan_planner_interface");
	ros::NodeHandle nh("~");


	// Read from the text file
	std::string plannerChoice;
	std::ifstream myPlannerFile("/home/crackoverflow/catkin_ws/ROSPlan/src/rosplan/rosplan_planning_system/src/planner.txt");
	getline(myPlannerFile, plannerChoice);
	myPlannerFile.close();

	switch(stoi(plannerChoice))
	{
		case 1:
		{
			KCL_rosplan::UPMPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 2:
		{
			KCL_rosplan::POPFPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 3:
		case 4:
		{
			KCL_rosplan::FFPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 5:
		{
			KCL_rosplan::OnlinePlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 6:
		{
			KCL_rosplan::TFDPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 7:
		{
			KCL_rosplan::CHIMPPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 8:
		{
			KCL_rosplan::SMTPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 9:
		{
			KCL_rosplan::RDDLSIMPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 10:
		{
			KCL_rosplan::PPRPPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 11:
		{
			KCL_rosplan::PANDAPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 12:
		{
			KCL_rosplan::LPGPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
		case 13:
		{
			KCL_rosplan::FDPlannerInterface pi(nh);
			runPlanner(pi, nh);
			break;
		}
	}

	return 0;
}
