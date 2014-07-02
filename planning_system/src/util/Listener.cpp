#include "ros/ros.h"
#include "planning_dispatch_msgs/ActionDispatch.h"
#include "planning_dispatch_msgs/ActionFeedback.h"

	/* print output of actionDispatch topic to system */
	void testDisptachCallback(const planning_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		ROS_INFO("Dispatch [%i, %s, %f]", msg->action_id, msg->name.c_str(), msg->duration);
		for(size_t i=0;i<msg->parameters.size();i++)
			ROS_INFO("\t%s:%s", msg->parameters[i].key.c_str(), msg->parameters[i].value.c_str());
	}

	/* print output of actionFeedback topic to system */
	void testFeedbackCallback(const planning_dispatch_msgs::ActionFeedback::ConstPtr& msg)
	{
		ROS_INFO("Feedback [%i,%s]", msg->action_id, msg->status.c_str());
		for(size_t i=0;i<msg->information.size();i++)
			ROS_INFO("\t%s:%s", msg->information[i].key.c_str(), msg->information[i].value.c_str());
	}

	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "kcl_planning_dispatch_listener");
		ros::NodeHandle n;
		ros::Subscriber dispatchSub = n.subscribe("/kcl_rosplan/action_dispatch", 1000, testDisptachCallback);
		ros::Subscriber feedbackSub = n.subscribe("/kcl_rosplan/action_feedback", 1000, testFeedbackCallback);
		ros::spin();

		return 0;
	}
