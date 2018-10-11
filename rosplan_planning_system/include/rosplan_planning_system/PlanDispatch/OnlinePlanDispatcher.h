//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#ifndef ROSPLAN_PLANNING_SYSTEM_ONLINEPLANDISPATCHER_H
#define ROSPLAN_PLANNING_SYSTEM_ONLINEPLANDISPATCHER_H

#include "PlanDispatcher.h"
#include "rosplan_dependencies/ippc_server.h"
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetRDDLParams.h>
#include <regex>

namespace KCL_rosplan
{

    class OnlinePlanDispatcher : public PlanDispatcher {
    private:

        // time plan was recevied
        double mission_start_time;
        rosplan_dispatch_msgs::CompletePlan current_plan;

        /* check preconditions are true */
        bool checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg);
        ros::ServiceClient queryKnowledgeClient;
        ros::ServiceClient queryDomainClient;
        ros::ServiceClient queryPropositionsClient;

        /* current action to dispatch */
        int current_action;
        int server_port_;

        boost::shared_ptr<XMLServer_t> ippcserver_ptr_;
        ros::ServiceClient get_rddl_params;
        ros::Publisher plan_publisher;

        rosplan_dispatch_msgs::ActionDispatch toActionMsg(std::string action_str, int action_id);
    public:

        /* constructor */
        explicit OnlinePlanDispatcher(ros::NodeHandle& nh);
        ~OnlinePlanDispatcher() = default;


        void reset() override;

        bool dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) override;
        void dispatchPlanActionlib() override;
        bool dispatchPlan(double missionStartTime, double planStartTime) override;

        void dispatchOneAction(rosplan_dispatch_msgs::ActionDispatch current_action_msg, double missionStartTime, double planStartTime);

        void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) override;



    };
}


#endif //ROSPLAN_PLANNING_SYSTEM_ONLINEPLANDISPATCHER_H
