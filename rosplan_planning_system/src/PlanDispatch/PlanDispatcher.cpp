//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include "rosplan_planning_system/PlanDispatch/PlanDispatcher.h"

namespace KCL_rosplan {
    PlanDispatcher::PlanDispatcher(ros::NodeHandle& nh): as_(nh, "dispatch_plan_action", false) {
        node_handle = &nh;

        kb_ = "knowledge_base";
        node_handle->getParam("knowledge_base", kb_);

        std::stringstream ss;
        ss << "/" << kb_ << "/query_state";
        queryKnowledgeClient = node_handle->serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>(ss.str());
        ss.str("");
        ss << "/" << kb_ << "/domain/operator_details";
        queryDomainClient = node_handle->serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str());
        ss.str("");
        ss << "/" << kb_ << "/state/goals";
        get_goals = node_handle->serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());
        ss.str("");

        // Register actionlib callbacks
        as_.registerGoalCallback(boost::bind(&KCL_rosplan::PlanDispatcher::dispatchPlanActionlib, this));
        as_.registerPreemptCallback(boost::bind(&KCL_rosplan::PlanDispatcher::cancelDispatch, this));
        as_.start(); // Start the action

        // start the plan parsing services
        service1 = nh.advertiseService("dispatch_plan", &KCL_rosplan::PlanDispatcher::dispatchPlanService, this);
        service2 = nh.advertiseService("cancel_dispatch", &KCL_rosplan::PlanDispatcher::cancelDispatchService, this);
        dispatching = false;


        // Action topics
        action_dispatch_topic = "action_dispatch";
        action_feedback_topic = "action_feedback";
        nh.getParam("action_dispatch_topic", action_dispatch_topic);
        nh.getParam("action_feedback_topic", action_feedback_topic);
        action_dispatch_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionDispatch>(action_dispatch_topic, 1, true);
        action_feedback_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionFeedback>(action_feedback_topic, 1, true);
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


    /*--------------------*/
    /* Dispatch interface */
    /*--------------------*/

    /**
     * plan dispatch service method (1)
     * dispatches plan as a service
     * @returns True iff every action was dispatched and returned success.
     */
    bool PlanDispatcher::dispatchPlanService(rosplan_dispatch_msgs::DispatchService::Request &req, rosplan_dispatch_msgs::DispatchService::Response &res) {
        if (dispatching) return false;
        dispatching = true;
        bool success = dispatchPlan(mission_start_time,ros::Time::now().toSec());
        dispatching = false;
        reset();
        res.success = success;
        res.goal_achieved = goalAchieved();
        return true;
    }

    /**
     * plan dispatch action method (2)
     * dispatches plan as a service
     * @returns True iff every action was dispatched and returned success.
     */
    void PlanDispatcher::dispatchPlanActionlib() {
        if (as_.isActive() or dispatching) {
            ROS_WARN("KCL: (%s) Got a new dispatch request but a plan is already being dispatched! It will be ignored", ros::this_node::getName().c_str());
        }
        else {
            as_.acceptNewGoal();
            dispatching = true;
            bool success = dispatchPlan(mission_start_time, ros::Time::now().toSec());
            dispatching = false;
            reset();
            rosplan_dispatch_msgs::NonBlockingDispatchResult res;
            res.success = success;
            res.goal_achieved = goalAchieved();
            as_.setSucceeded(res);
        }
    }

    void PlanDispatcher::reset() {
        replan_requested = false;
        dispatch_paused = false;
        plan_cancelled = false;
        action_received.clear();
        action_completed.clear();
        plan_received = false;
        dispatching = false;
    }

    /**
	 *	Returns true of the actions preconditions are true in the current state. Calls the Knowledge Base.
	 */
    bool PlanDispatcher::checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg) {

        // get domain operator details
        rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
        srv.request.name = msg.name;
        if(!queryDomainClient.call(srv)) {
            ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s", ros::this_node::getName().c_str(), msg.name.c_str());
            return false;
        }

        // setup service call
        rosplan_knowledge_msgs::DomainOperator op = srv.response.op;
        rosplan_knowledge_msgs::KnowledgeQueryService positiveQuerySrv;

        // iterate through conditions
        std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit = op.at_start_simple_condition.begin();
        for(; cit!=op.at_start_simple_condition.end(); cit++) {

            // create condition
            rosplan_knowledge_msgs::KnowledgeItem condition;
            condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            condition.attribute_name = cit->name;

            // populate parameters
            for(int i=0; i<cit->typed_parameters.size(); i++) {

                // set parameter label to predicate label
                diagnostic_msgs::KeyValue param;
                param.key = cit->typed_parameters[i].key;

                // search for correct operator parameter value
                for(int j=0; j<msg.parameters.size() && j<op.formula.typed_parameters.size(); j++) {
                    if(op.formula.typed_parameters[j].key == cit->typed_parameters[i].key) {
                        param.value = msg.parameters[j].value;
                    }
                }
                condition.values.push_back(param);
            }
            positiveQuerySrv.request.knowledge.push_back(condition);
        }

        // checking negative preconditions
        cit = op.at_start_neg_condition.begin();
        // flag to indicate that at least one of the negative conditions was found in KB
        bool neg_preconditions = false;
        rosplan_knowledge_msgs::KnowledgeQueryService negativeQuerySrv;
        for(; cit!=op.at_start_neg_condition.end(); cit++) {
            // create condition
            rosplan_knowledge_msgs::KnowledgeItem condition;
            condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            condition.attribute_name = cit->name;

            // populate parameters
            for(int i=0; i<cit->typed_parameters.size(); i++) {

                // set parameter label to predicate label
                diagnostic_msgs::KeyValue param;
                param.key = cit->typed_parameters[i].key;

                // search for correct operator parameter value
                for(int j=0; j<msg.parameters.size() && j<op.formula.typed_parameters.size(); j++) {
                    if(op.formula.typed_parameters[j].key == cit->typed_parameters[i].key) {
                        param.value = msg.parameters[j].value;
                    }
                }
                condition.values.push_back(param);
            }
            negativeQuerySrv.request.knowledge.push_back(condition);
        }

        // check negative conditions in knowledge base (conditions that should not be present in KB)
        if (queryKnowledgeClient.call(negativeQuerySrv)) {
            // iterate over results to check if at least one fact was found in KB
            for(auto it = negativeQuerySrv.response.results.begin(); it != negativeQuerySrv.response.results.end(); it++)
                if (*it) {
                    // rise flag to indicate that at least one neg condition was not achieved
                    neg_preconditions = true;

                    // print which negative condition was found in KB
                    std::stringstream ss;
                    // get index of failed neg precondition
                    int index = std::distance(it, negativeQuerySrv.response.results.begin());
                    ss << negativeQuerySrv.request.knowledge[index].attribute_name;
                    auto pit = negativeQuerySrv.request.knowledge[index].values.begin();
                    for(; pit != negativeQuerySrv.request.knowledge[index].values.end(); pit++) {
                        ss << " ";
                        ss << pit->value.c_str();
                    }

                    ROS_INFO("Negative precondition not achieved : not (%s)", ss.str().c_str());
                }
        } else {
            ROS_ERROR("KCL: (%s) Failed to call service query_state", ros::this_node::getName().c_str());
        }

        // check positive conditions in knowledge base
        if (queryKnowledgeClient.call(positiveQuerySrv)) {

            if(!positiveQuerySrv.response.all_true) {
                std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kit;
                for(kit=positiveQuerySrv.response.false_knowledge.begin(); kit != positiveQuerySrv.response.false_knowledge.end(); kit++)
                    ROS_INFO("KCL: (%s) Precondition not achieved: %s", ros::this_node::getName().c_str(), kit->attribute_name.c_str());
            }
            return positiveQuerySrv.response.all_true && !neg_preconditions;

        } else {
            ROS_ERROR("KCL: (%s) Failed to call service query_state", ros::this_node::getName().c_str());
        }
    }

    bool PlanDispatcher::goalAchieved() {
        rosplan_knowledge_msgs::GetAttributeService goal;
        if (not get_goals.call(goal)) {
            ROS_ERROR("KCL: (%s) Failed to call service get_goals", ros::this_node::getName().c_str());
            return false;
        }
        if (goal.response.attributes.empty()) return true;
        rosplan_knowledge_msgs::KnowledgeQueryService querySrv;
        querySrv.request.knowledge = goal.response.attributes;
        if (not queryKnowledgeClient.call(querySrv)) {
            ROS_ERROR("KCL: (%s) Failed to call service query_state", ros::this_node::getName().c_str());
            return false;
        }
        return querySrv.response.all_true;
    }

}
