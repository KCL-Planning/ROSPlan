/**
 * This file describes a class that dispatches a plan.
 */
#include <rosplan_dispatch_msgs/CompletePlan.h>
#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/NonBlockingDispatchAction.h"
#include "std_srvs/Empty.h"
#include <actionlib/server/simple_action_server.h>

#ifndef KCL_dispatcher
#define KCL_dispatcher

namespace KCL_rosplan
{

	class PlanDispatcher
	{
	protected:
		actionlib::SimpleActionServer<rosplan_dispatch_msgs::NonBlockingDispatchAction> as_;
		ros::ServiceServer service1;
		ros::ServiceServer service2;

		ros::NodeHandle* node_handle;

		std::string action_dispatch_topic;
		std::string action_feedback_topic;

		/* dispatch state */
		bool plan_received;
		std::map<int,bool> action_received;
		std::map<int,bool> action_completed;

		/* dispatch modes */
		bool dispatch_paused;
		bool replan_requested;
		bool plan_cancelled;
		bool dispatching;

        /* action publishers */
        ros::Publisher action_dispatch_publisher;
        ros::Publisher action_feedback_publisher;
	public:
	    PlanDispatcher(ros::NodeHandle& nh);
	    ~PlanDispatcher() = default;

		/* control callback */
		bool cancelDispatch();
		bool cancelDispatchService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

		virtual void reset() =0;

		/* plan dispatch methods */
		virtual bool dispatchPlan(double missionStartTime, double planStartTime) =0;
		virtual bool dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)=0;
		virtual void dispatchPlanActionlib()=0;

		void publishFeedback(const rosplan_dispatch_msgs::ActionFeedback& fb);


		/* action feedback methods */
		virtual void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) =0;


	};
}

#endif
