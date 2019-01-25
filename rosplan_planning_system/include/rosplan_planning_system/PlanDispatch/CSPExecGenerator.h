/*
 * Copyright [2019] <KCL - ISR collaboration>
 *
 * KCL: King's College London
 * ISR: Institue for Systems and Robotics
 *
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk), Oscar Lima (olima_84@yahoo.com)
 *
 * Finds out different options for a plan to be executed.
 *
 */

#ifndef CSP_EXEC_GENERATOR_NODE_H
#define CSP_EXEC_GENERATOR_NODE_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_dispatch_msgs/EsterelPlan.h>
#include <rosplan_dispatch_msgs/EsterelPlanArray.h>
#include <rosplan_dispatch_msgs/EsterelPlanNode.h>
#include <rosplan_dispatch_msgs/EsterelPlanEdge.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include "rosplan_action_interface/ActionSimulator.h"

class CSPExecGenerator
{
    public:
        CSPExecGenerator();
        ~CSPExecGenerator();

        // get parameters from param server
        void getParams();

        // callback for event_in received msg
        void esterelPlanCB(const rosplan_dispatch_msgs::EsterelPlan::ConstPtr& msg);

        // generate plan alternatives based on search
        bool compute_exec_alternatives();

        // service callback
        bool srvCB(std_srvs::SetBool::Response &req, std_srvs::SetBool::Response &res);

        // ros node main loop
        void update();

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_set_of_solutions_;
        ros::Subscriber sub_esterel_plan_;

        // flag used to know when we have received a callback
        bool is_esterel_plan_received_;

        // stores the received msg in esterel plan callback
        rosplan_dispatch_msgs::EsterelPlan esterel_plan_msg_;

        // plan execution alternatives, each one is a fully ordered esterel plan
        rosplan_dispatch_msgs::EsterelPlanArray solution_set_;

        // to simulate actions in a private (own) KB
        ActionSimulator action_simulator_;
};
#endif  // CSP_EXEC_GENERATOR_NODE_H
