#include "rosplan_planning_system/PlanDispatch/PlanDispatcher.h"

namespace KCL_rosplan {

	/*--------------------*/
	/* Dispatch interface */
	/*--------------------*/

	/**
	 * plan dispatch service method (1) 
	 * dispatches plan as a service
	 */
	bool PlanDispatcher::dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
		ROS_INFO("KCL: (%s) Dispatching plan as service.", ros::this_node::getName().c_str());
		reset();
		bool success = dispatchPlan(ros::WallTime::now().toSec(),ros::WallTime::now().toSec());
		return true;
	}
} // close namespace
