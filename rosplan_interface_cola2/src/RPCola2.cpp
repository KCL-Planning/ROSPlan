#include "rosplan_interface_cola2/RPCola2.h"

/* The implementation of RPCola2.h */
namespace KCL_rosplan {

	/* constructor */
	RPCola2::RPCola2(ros::NodeHandle &nh, std::string &actionserver)
			: message_store(nh), action_client(actionserver, true) {
		
		// create the action client
		ROS_INFO("KCL: (Cola2) waiting for server to start on %s", actionserver.c_str());
		action_client.waitForServer();
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

		planning_pilot_id_offset = 2;
	}

	/* action dispatch callback */
	void RPCola2::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-goto-waypoint actions
		if(0!=msg->name.compare("goto_waypoint")) return;

		ROS_INFO("KCL: (Cola2) action recieved");

		// get waypoint ID from action dispatch
		std::string wpID;
		bool found = false;
		for(size_t i=0; i<msg->parameters.size(); i++) {
			if(0==msg->parameters[i].key.compare("to")) {
				wpID = msg->parameters[i].value;
				found = true;
			}
		}
		if(!found) {
			ROS_INFO("KCL: (Cola2) aborting action dispatch; malformed parameters");
			return;
		}
		
		// get pose from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (Cola2) aborting action dispatch; no matching wpID %s", wpID.c_str());
				return;
			}
			if(results.size()>1)
				ROS_INFO("KCL: (Cola2) multiple waypoints share the same wpID");

			// prepare cola2 action
			cola2_control::WorldWaypointReqGoal goal;
			goal.goal.requester = "rosplan_interface_cola2";
			goal.goal.id = planning_pilot_id_offset + msg->action_id;
			goal.goal.priority = auv_msgs::GoalDescriptor::PRIORITY_NORMAL;
			goal.altitude_mode = false;
			goal.position.north = results[0]->pose.position.x;
			goal.position.east = results[0]->pose.position.y;
			goal.position.depth = results[0]->pose.position.z;
			goal.altitude = results[0]->pose.position.z;
			goal.orientation.roll = 0.0;
			goal.orientation.pitch = 0.0;
			goal.orientation.yaw = results[0]->pose.orientation.w;

			// choose the movement
			goal.mode = "waypoint";
			goal.disable_axis.x = false;
			goal.disable_axis.y = true;
			goal.disable_axis.z = false;
			goal.disable_axis.roll = true;
			goal.disable_axis.pitch = true;
			goal.disable_axis.yaw = false;

			// Set tolerance
			goal.position_tolerance.x = 1.0;
			goal.position_tolerance.y = 1.0;
			goal.position_tolerance.z = 1.0;
			goal.orientation_tolerance.roll = 1.00;
			goal.orientation_tolerance.pitch = 1.00;
			goal.orientation_tolerance.yaw = 1.0;
			goal.timeout = 0.0;

			// send goal
			action_client.sendGoal(goal);

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);

			bool finished_before_timeout = action_client.waitForResult(ros::Duration(5*msg->duration));
			if (finished_before_timeout) {

				actionlib::SimpleClientGoalState state = action_client.getState();
				ROS_INFO("KCL: (Cola2) action finished: %s", state.toString().c_str());
				
				// publish feedback (achieved)
				 rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = msg->action_id;
				fb.status = "action achieved";
				action_feedback_pub.publish(fb);

			} else {

				// publish feedback (failed)
				 rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);

				ROS_INFO("KCL: (Cola2) action timed out");

			}

		} else ROS_INFO("KCL: (Cola2) aborting action dispatch; query to sceneDB failed");
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_cola2");
		ros::NodeHandle nh;

		std::string actionserver;
		nh.param("goto_server", actionserver, std::string("/absolute_movement"));

		// create PDDL action subscriber
		KCL_rosplan::RPCola2 rpc2(nh, actionserver);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPCola2::dispatchCallback, &rpc2);
		ROS_INFO("KCL: (Cola2) Ready to receive");

		ros::spin();
		return 0;
	}
