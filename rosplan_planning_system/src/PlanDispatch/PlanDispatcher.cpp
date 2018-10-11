//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#include <rosplan_planning_system/PlanDispatch/PlanDispatcher.h>

#include "rosplan_planning_system/PlanDispatch/PlanDispatcher.h"

namespace KCL_rosplan {
    PlanDispatcher::PlanDispatcher(ros::NodeHandle& nh): as_(nh, "dispatch_plan_action", false) {
        node_handle = &nh;

        as_.registerGoalCallback(boost::bind(&KCL_rosplan::PlanDispatcher::dispatchPlanAction, this));
        as_.registerPreemptCallback(boost::bind(&KCL_rosplan::PlanDispatcher::cancelDispatch, this));
        as_.start(); // Start the action

        // start the plan parsing services
        service1 = nh.advertiseService("dispatch_plan", &KCL_rosplan::PlanDispatcher::dispatchPlanService, this);
        service2 = nh.advertiseService("cancel_dispatch", &KCL_rosplan::PlanDispatcher::cancelDispatchService, this);
        dispatching = false;
    }

    void PlanDispatcher::publishFeedback(const rosplan_dispatch_msgs::ActionFeedback &fb) {
        if (as_.isActive()) { // Action server is the one dispatching
            rosplan_dispatch_msgs::NonBlockingDispatchFeedback afeedback;
            afeedback.feedback = fb;
            as_.publishFeedback(afeedback);
        }
        // Publish feedback in topic anyway...
        action_feedback_publisher.publish(fb);
    }

    bool PlanDispatcher::cancelDispatch() {
        if (as_.isNewGoalAvailable()) { // Preempt caused by sending of new goal, ignore it
            //ROS_WARN("KCL: (%s) Got a new dispatch request but a plan is already being dispatched!", ros::this_node::getName().c_str());
            // Don't cancel a goal as it is being preempted because of a new goal sent
            return false;
        }
        ROS_INFO("KCL: (%s) Cancel plan command received.", ros::this_node::getName().c_str());
        if (as_.isActive()) as_.setPreempted();
        plan_cancelled = true;
        return true;
    }

    bool PlanDispatcher::cancelDispatchService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        return cancelDispatch();
    }

}