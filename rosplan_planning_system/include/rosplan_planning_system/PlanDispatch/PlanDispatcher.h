/**
 * This file describes a class that dispatches a plan.
 */


#ifndef KCL_dispatcher
#define KCL_dispatcher

#include <rosplan_dispatch_msgs/CompletePlan.h>
#include "ros/ros.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/NonBlockingDispatchAction.h"
#include "std_srvs/Empty.h"
#include <actionlib/server/simple_action_server.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include "rosplan_dispatch_msgs/DispatchService.h"

namespace KCL_rosplan
{

	class PlanDispatcher
	{
	protected:
		actionlib::SimpleActionServer<rosplan_dispatch_msgs::NonBlockingDispatchAction> as_;
		ros::ServiceServer service1;
		ros::ServiceServer service2;

        double mission_start_time;

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

        /* check preconditions are true */
        bool checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg);
        ros::ServiceClient queryKnowledgeClient;
        ros::ServiceClient queryDomainClient;
        ros::ServiceClient get_goals;

        bool goalAchieved(); // Returns true if the goal was achieved

        std::string kb_; // knowledge base name
    public:
	    explicit PlanDispatcher(ros::NodeHandle& nh);
	    ~PlanDispatcher() = default;

		/* control callback */
		bool cancelDispatch();
		bool cancelDispatchService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

		virtual void reset() =0;

		/* plan dispatch methods */
		virtual bool dispatchPlan(double missionStartTime, double planStartTime) =0;
		virtual bool dispatchPlanService(rosplan_dispatch_msgs::DispatchService::Request &req, rosplan_dispatch_msgs::DispatchService::Response &res);
		virtual void dispatchPlanActionlib();

		/* Publishes the actionfeedback */
		void publishFeedback(const rosplan_dispatch_msgs::ActionFeedback& fb);

		/* action feedback methods */
		virtual void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) =0;


	};
}

#endif
