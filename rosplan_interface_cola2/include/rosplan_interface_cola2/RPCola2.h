#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "cola2_control/WorldWaypointReqAction.h"
#include "auv_msgs/GoalDescriptor.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"

#ifndef KCL_cola2
#define KCL_cola2

/**
 * This file defines the RPCola2 class.
 * RPCola2 is used to connect ROSPlan to the Cola2 library.
 * PDDL "goto_waypoint" actions become "cola2_control::WorldWaypointReq" msgs.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPCola2
	{

	private:

		int planning_pilot_id_offset;
		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<cola2_control::WorldWaypointReqAction> action_client;
		ros::Publisher waypoint_request_pub;
		ros::Publisher action_feedback_pub;

	public:

		/* constructor */
		RPCola2(ros::NodeHandle &nh, std::string &serverName);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		/* listen to and process "/absolute_movement/result" topic */
		void movementCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
