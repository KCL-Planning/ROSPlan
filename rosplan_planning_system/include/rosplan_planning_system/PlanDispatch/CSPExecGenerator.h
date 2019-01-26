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

        /**
         * @brief constructor, empty
         */
        CSPExecGenerator();

        /**
         * @brief destructor, shutdown publishers and subscribers
         */
        ~CSPExecGenerator();

        /**
         * @brief callback to receive the fully ordered esterel plan
         * @param msg the plan is received in this variable, is written by reference
         */
        void esterelPlanCB(const rosplan_dispatch_msgs::EsterelPlan::ConstPtr& msg);

        /**
         * @brief remove conditional edges from a fully connected esterel plan received in original_fully_ordered_plan_
         * @param plan the plan from which the conditional edges need to be removed
         * @return a partially ordered plan with only interference edges in EsterelPlan format
         */
        rosplan_dispatch_msgs::EsterelPlan removeConditionalEdges(rosplan_dispatch_msgs::EsterelPlan plan);

        /**
         * @brief generate plan alternatives based on search
         * @return true if succeeded
         */
        bool computeExecAlternatives();

        /**
         * @brief service callback with user request to generate execution alternatives
         * @param req input from user gets received in this variable
         * @param res service response gets written here by reference, true if at least
         * one possible execution was found, false if replan is needed (means plan is invalid)
         * @return true if service finished execution, false otherwise
         */
        bool srvCB(std_srvs::SetBool::Response &req, std_srvs::SetBool::Response &res);

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_set_of_solutions_, pub_pop_;
        ros::Subscriber sub_esterel_plan_;

        /// flag used to know when we have received a callback
        bool is_esterel_plan_received_;

        /// stores the received msg in esterel plan callback
        rosplan_dispatch_msgs::EsterelPlan original_fully_ordered_plan_;

        /// to store the partial ordered plan, only with interference edges
        rosplan_dispatch_msgs::EsterelPlan partial_order_plan_;

        /// plan execution alternatives, each one is a fully ordered esterel plan
        rosplan_dispatch_msgs::EsterelPlanArray solution_set_;

        /// to simulate actions in a private (own) KB
        ActionSimulator action_simulator_;
};
#endif  // CSP_EXEC_GENERATOR_NODE_H
