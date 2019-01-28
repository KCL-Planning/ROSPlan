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
#include <algorithm>

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
         * @brief init C : set of temporal constraints
         * @param set_of_constraints return value gets written here by reference
         */
        void initConstraints(std::map<int, int> &set_of_constraints);

        /**
         * @brief check if set of ordered nodes satisfies the set of constraints
         * @param ordered_nodes F or set of ordered nodes
         * @param set_of_constraints C or set of constraints
         * @return true if the set of nodes satisfy the set of constraints, false otherwise
         */
        bool checkTemporalConstraints(std::vector<int> &set_of_ordered_nodes,
            std::map<int, int> &set_of_constraints);

        /**
         * @brief check if curent state (P) satisfies all goals proposed in (G)
         * @param as action simulator object, it contains a method to check if goals ae achieved
         */
        bool areGoalsAchieved(ActionSimulator &as);

        /**
         * @brief iterate over open list (O), check if node preconditions are met in current state (P)
         * @param open_list set of unordered nodes
         * @return set of nodes (V) which preconditions are met in current state (P)
         */
        std::vector<rosplan_dispatch_msgs::EsterelPlanNode> validNodes(
            std::vector<rosplan_dispatch_msgs::EsterelPlanNode> open_list);

        /**
         * @brief shift nodes from open list (O) to ordered plans (R) offering different execution alternatives
         * @return true if at least one valid execution was found, false otherwise
         */
        bool orderNodes();

        /**
         * @brief generate plan alternatives based on search
         * @return true if succeeded
         */
        bool generateFullyConnectedPlan();

        /**
         * @brief service callback with user request to generate execution alternatives
         * @param req input from user gets received in this variable
         * @param res service response gets written here by reference, true if at least
         * one possible execution was found, false if replan is needed (means plan is invalid)
         * @return true if service finished execution, false otherwise
         */
        bool srvCB(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

        /**
         * @brief wait for callbacks to arrive
         */
        void update();

        // remove
        void testCheckTemporalConstraints();

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_set_of_solutions_, pub_pop_;
        ros::Subscriber sub_esterel_plan_;
        ros::ServiceServer srv_gen_alternatives_;

        /// flag used to know when we have received a callback
        bool is_esterel_plan_received_;

        /// stores the received msg in esterel plan callback
        rosplan_dispatch_msgs::EsterelPlan original_plan_;

        /// P: to simulate actions in a private (own) KB
        ActionSimulator *action_simulator_;

        /// O: open list, the list of nodes id's to order
        std::vector<int> open_list_;

        /// C: set of constraints
        std::map<int, int> set_of_constraints_;

        // F: store the set of ordered nodes id's, empty at the beginning
        std::vector<int> ordered_nodes_;

        /// R: plan execution alternatives, each one is a fully ordered esterel plan
        rosplan_dispatch_msgs::EsterelPlanArray ordered_plans_;
};
#endif  // CSP_EXEC_GENERATOR_NODE_H
