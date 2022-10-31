//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#ifndef ROSPLAN_PLANNING_SYSTEM_ONLINEPLANDISPATCHER_H
#define ROSPLAN_PLANNING_SYSTEM_ONLINEPLANDISPATCHER_H

#include "PlanDispatcher.h"
#include "rosplan_dependencies/ippc_server.h"
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetRDDLParams.h>
#include <rosplan_knowledge_msgs/GetRDDLImmediateReward.h>
#include <regex>

namespace KCL_rosplan
{

    class OnlinePlanDispatcher : public PlanDispatcher {
    private:
        // time plan was recevied
        rosplan_dispatch_msgs::CompletePlan current_plan;

        ros::ServiceClient queryPropositionsClient;
        ros::ServiceClient get_rddl_params;
        ros::ServiceClient get_planner_params;
        ros::ServiceClient get_immediate_reward;

        /* current action to dispatch */
        int current_action;
        int server_port_;
        bool compute_rewards_;

        ros::Publisher plan_publisher;

        std::vector<rosplan_dispatch_msgs::ActionDispatch> toActionMsg(std::string action_str, int action_id);
    public:

        /* constructor */
        explicit OnlinePlanDispatcher(ros::NodeHandle& nh);
        ~OnlinePlanDispatcher();


        void reset() override;

        bool dispatchPlanService(rosplan_dispatch_msgs::DispatchService::Request &req, rosplan_dispatch_msgs::DispatchService::Response &res) override;
        void dispatchPlanActionlib() override;
        bool dispatchPlan(double missionStartTime, double planStartTime) override;

        void dispatchOneAction(rosplan_dispatch_msgs::ActionDispatch current_action_msg, double missionStartTime, double planStartTime);

        void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) override;
    };
}


#endif //ROSPLAN_PLANNING_SYSTEM_ONLINEPLANDISPATCHER_H
