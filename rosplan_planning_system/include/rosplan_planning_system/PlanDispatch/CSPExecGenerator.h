/*
 * Copyright [2019] <KCL - ISR collaboration>
 *
 * KCL: King's College London
 * ISR: Institue for Systems and Robotics
 *
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk), Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Finds out different options for a plan to be executed.
 *
 */

#ifndef CSP_EXEC_GENERATOR_NODE_H
#define CSP_EXEC_GENERATOR_NODE_H

#include <ros/ros.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_dispatch_msgs/EsterelPlan.h>
#include <rosplan_dispatch_msgs/EsterelPlanNode.h>
#include <rosplan_dispatch_msgs/EsterelPlanEdge.h>
#include <rosplan_dispatch_msgs/EsterelPlanArray.h>
#include <rosplan_dispatch_msgs/ExecAlternatives.h>
#include <diagnostic_msgs/KeyValue.h>
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
         * @brief easy print vector nodes
         * @param msg the name of the node list, e.g. "open list"
         * @param nodes the node list
         */
        void printNodes(std::string msg, std::vector<int> &nodes);

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
         * @brief example of how to use the functions of this class
         */
        void testFunctions();

        /**
         * @brief check if curent state (P) satisfies all goals proposed in (G)
         * @param as action simulator object, it contains a method to check if goals ae achieved
         */
        bool areGoalsAchieved(ActionSimulator &as);

        /**
         * @brief input a node id and return the action name and params
         * @param action_id an integer identifying the action by id
         * @param action_name return value is written here by reference
         * @param params return value is written here by reference
         * @param plan the plan from where to extract the actions name and params
         * @return true if node is action start, false if node is action end
         */
        bool getAction(int action_id, std::string &action_name, std::vector<std::string> &params,
            rosplan_dispatch_msgs::EsterelPlan &plan, bool &action_start);

        /**
         * @brief receive as input a node id from a end action node,
         * return by reference the node id of the correspondent action start
         * @param end_node_id input node id, needs to be an action end id
         * @param action_start_node_id return value is written here by reference
         * @return true if function was able to find correspondent start action node id, false if it was unable
         */
        bool getStartNodeID(int end_node_id, int &action_start_node_id);

        /**
         * @brief iterate over open list (O), check if node preconditions are met in current state (P)
         * @param open_list set of unordered nodes
         * @param valid_nodes return value gets written here by reference, list of nodes
         * @return true if valid nodes were found, false otherwise
         */
        bool validNodes(std::vector<int> &open_list, std::vector<int> &valid_nodes);

        /**
         * @brief find all nodes b in open list (O) which ordering constraints enforce them before a
         * @param a integer representing the node id, which is the reference for this function
         * @param open_list the list of nodes which have not yet being ordered
         * @return a list of nodes b ordered before a, e.g. open_list = {1,4,3,2}, a = 3, return = {1, 4}
         */
        std::vector<int> findNodesBeforeA(int a, std::vector<int> &open_list);

        /**
         * @brief backtrack: popf, remove last element from f, store in variable and revert that action
         * @param reason_for_backtrack used for log information and feedback to user
         */
        void backtrack(std::string reason_for_backtrack);

        /**
         * @brief shift nodes from open list (O) to ordered plans (R) offering different execution alternatives
         * @param open_list the list of nodes which have not yet being ordered, at startup is composed of all nodes
         * in the partially ordered plan, later the game is take from it the applicable nodes and pass them to
         * the ordered list (which is used also as stack for the DFS search with backtrack)
         * @return true if at least one valid execution was found, false otherwise
         */
        bool orderNodes(std::vector<int> open_list);

        /**
         * @brief generate plan alternatives based on search
         * @return true if succeeded
         */
        bool generatePlans();

        /**
         * @brief iterate over the edges of the received esterel plan and delete all conditional edges
         * @param esterel_plan the plan from which you want to remove its conditional edges
         * @return esterel plan output, a modified version of the input plan, without conditional edges
         */
        rosplan_dispatch_msgs::EsterelPlan removeConditionalEdges(rosplan_dispatch_msgs::EsterelPlan &esterel_plan);

        /**
         * @brief add constrains to the partially ordered plan (esterel plan without conditional edges)
         * @param ordered_nodes input, a totally odered list of nodes, which contains one possible way of executing the plan
         * @return esterel plan msg
         */
        rosplan_dispatch_msgs::EsterelPlan convertListToEsterel(std::vector<int> &ordered_nodes);

        /**
         * @brief service callback with user request to generate execution alternatives
         * @param req input from user gets received in this variable
         * @param res service response gets written here by reference, true if at least
         * one possible execution was found, false if replan is needed (means plan is invalid)
         * @return true if service finished execution, false otherwise
         */
        bool srvCB(rosplan_dispatch_msgs::ExecAlternatives::Request& req,
                   rosplan_dispatch_msgs::ExecAlternatives::Response& res);

        /**
         * @brief wait for callbacks to arrive
         */
        void update();

    private:
        // ros related variables
        ros::NodeHandle nh_;
        ros::Publisher pub_valid_plans_, pub_esterel_plan_;
        ros::Subscriber sub_esterel_plan_;
        ros::ServiceServer srv_gen_alternatives_;

        /// flag used to know when we have received a callback
        bool is_esterel_plan_received_;

        /// stores the received msg in esterel plan callback
        rosplan_dispatch_msgs::EsterelPlan original_plan_, partial_order_plan_;

        /// P: to simulate actions in a private (own) KB
        ActionSimulator action_simulator_;

        /// C: set of constraints
        std::map<int, int> set_of_constraints_;

        // F: store the set of ordered nodes id's, empty at the beginning
        std::vector<int> ordered_nodes_;

        /// R: plan execution alternatives, each one is a fully ordered esterel plan
        std::vector<std::vector<int> > ordered_plans_;

        /// Information coming from the service call gets saved into member variable
        /// it tells which nodes are currently being/done executed
        std::vector<int> action_executing_;
};
#endif  // CSP_EXEC_GENERATOR_NODE_H
