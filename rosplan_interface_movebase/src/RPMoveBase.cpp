#include "rosplan_interface_movebase/RPMoveBase.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPMoveBase::RPMoveBase(ros::NodeHandle &nh, std::string &actionserver)
	 : message_store(nh), action_client(actionserver, true) {
		
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		// costmap client
		clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

		// create the action client
		ROS_INFO("KCL: (MoveBase) waiting for action server to start on %s", actionserver.c_str());
		action_client.waitForServer();
		
		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	}

	/* action dispatch callback */
	void RPMoveBase::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// ignore non-goto-waypoint actions
		if(0!=msg->name.compare("goto_waypoint")) return;

		ROS_INFO("KCL: (MoveBase) action recieved");

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
			ROS_INFO("KCL: (MoveBase) aborting action dispatch; malformed parameters");
			return;
		}
		
		// get pose from message store
		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
			if(results.size()<1) {
				ROS_INFO("KCL: (MoveBase) aborting action dispatch; no matching wpID %s", wpID.c_str());
				return;
			}
			if(results.size()>1)
				ROS_INFO("KCL: (MoveBase) multiple waypoints share the same wpID");

			// dispatch MoveBase action
			move_base_msgs::MoveBaseGoal goal;
			geometry_msgs::PoseStamped &pose = *results[0];
			goal.target_pose = pose;
			goal.target_pose.pose.orientation.w = 1;
			action_client.sendGoal(goal);

			// publish feedback (enabled)
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = msg->action_id;
			fb.status = "action enabled";
			action_feedback_pub.publish(fb);

			bool finished_before_timeout = action_client.waitForResult(ros::Duration(5*msg->duration));
			if (finished_before_timeout) {

				actionlib::SimpleClientGoalState state = action_client.getState();
				ROS_INFO("KCL: (MoveBase) action finished: %s", state.toString().c_str());

				if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {

					// remove old robot_at
					rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
					updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
					updatePredSrv.request.knowledge.attribute_name = "robot_at";
					diagnostic_msgs::KeyValue pair;
					pair.key = "v";
					pair.value = "kenny";
					updatePredSrv.request.knowledge.values.push_back(pair);
					update_knowledge_client.call(updatePredSrv);

					// predicate robot_at
					updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
					updatePredSrv.request.knowledge.attribute_name = "robot_at";
					diagnostic_msgs::KeyValue pairWP;
					pairWP.key = "wp";
					pairWP.value = wpID;
					updatePredSrv.request.knowledge.values.push_back(pairWP);
					update_knowledge_client.call(updatePredSrv);

					ros::Rate big_rate(0.5);
					big_rate.sleep();

					// publish feedback (achieved)
					rosplan_dispatch_msgs::ActionFeedback fb;
					fb.action_id = msg->action_id;
					fb.status = "action achieved";
					action_feedback_pub.publish(fb);

				} else {

					// clear costmaps
					std_srvs::Empty emptySrv;
					clear_costmaps_client.call(emptySrv);

					// publish feedback (failed)
					rosplan_dispatch_msgs::ActionFeedback fb;
					fb.action_id = msg->action_id;
					fb.status = "action failed";
					action_feedback_pub.publish(fb);
				}

			} else {

				action_client.cancelAllGoals();

				// publish feedback (failed)
				 rosplan_dispatch_msgs::ActionFeedback fb;
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub.publish(fb);

				ROS_INFO("KCL: (MoveBase) action timed out");

			}

		} else ROS_INFO("KCL: (MoveBase) aborting action dispatch; query to sceneDB failed");
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_movebase");
		ros::NodeHandle nh;

		std::string actionserver;
		nh.param("action_server", actionserver, std::string("/move_base"));

		// create PDDL action subscriber
		KCL_rosplan::RPMoveBase rpmb(nh, actionserver);
	
		// listen for action dispatch
		ros::Subscriber ds = nh.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::RPMoveBase::dispatchCallback, &rpmb);
		ROS_INFO("KCL: (MoveBase) Ready to receive");

		ros::spin();
		return 0;
	}
