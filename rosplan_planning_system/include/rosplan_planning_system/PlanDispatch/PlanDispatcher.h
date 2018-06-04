/**
 * This file describes a class that dispatches a plan.
 */
#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "std_srvs/Empty.h"

#ifndef KCL_dispatcher
#define KCL_dispatcher

namespace KCL_rosplan
{

	class PlanDispatcher
	{
	protected:

		ros::NodeHandle* node_handle;

		std::string action_dispatch_topic;
		std::string action_feedback_topic;

		/* dispatch state */
		bool plan_recieved;
		std::map<int,bool> action_received;
		std::map<int,bool> action_completed;

	public:

		/* control callback */
		bool cancelDispatchService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
			ROS_INFO("KCL: (%s) Cancel plan command recieved.", ros::this_node::getName().c_str());
			plan_cancelled = true;
			return true;
		}

		/* dispatch modes */
		bool dispatch_paused;
		bool replan_requested;
		bool plan_cancelled;

		virtual void reset() =0;

		/* plan dispatch methods */
		virtual bool dispatchPlan(double missionStartTime, double planStartTime) =0;
		virtual bool dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) =0;

		/* action feedback methods */
		virtual void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) =0;

		/* action publishers */
		ros::Publisher action_dispatch_publisher;
		ros::Publisher action_feedback_publisher;
	};
}

#endif
