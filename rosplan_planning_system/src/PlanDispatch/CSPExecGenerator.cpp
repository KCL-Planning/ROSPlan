/*
 * Copyright [2019] <KCL - ISR collaboration>
 *
 * KCL: King's College London
 * ISR: Institue for Systems and Robotics
 *
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk), Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Finds out many different alternatives for a esterel plan to be executed.
 *
 */

#include <rosplan_planning_system/PlanDispatch/CSPExecGenerator.h>

CSPExecGenerator::CSPExecGenerator() : nh_("~"), is_esterel_plan_received_(false)
{
    // subscriptions: subscribe to esterel plan, a fully ordered plan
    sub_esterel_plan_ = nh_.subscribe("/rosplan_parsing_interface/complete_plan", 1, &CSPExecGenerator::esterelPlanCB, this);

    // services: compute different execution alternatives from a partially ordered esterel plan (a plan
    // with no conditional edges, but only interference edges)
    srv_gen_alternatives_ = nh_.advertiseService("gen_exec_alternatives", &CSPExecGenerator::srvCB, this);

    // mirror KB (query real KB and get its data) but without facts and goals
    action_simulator_.init();
}

CSPExecGenerator::~CSPExecGenerator()
{
    // shut down publishers and subscribers
    sub_esterel_plan_.shutdown();
}

void CSPExecGenerator::printNodes(std::string msg, std::vector<int> &nodes)
{
    std::stringstream ss;
    for(auto nit=nodes.begin(); nit!=nodes.end(); nit++) {
        ss << *nit;
        ss << ",";
    }
    ROS_INFO("%s : {%s}", msg.c_str(), ss.str().c_str());
}

void CSPExecGenerator::esterelPlanCB(const rosplan_dispatch_msgs::EsterelPlan::ConstPtr& msg)
{
    ROS_INFO("esterel plan received");
    original_plan_ = *msg;
    is_esterel_plan_received_ = true;
}

void CSPExecGenerator::initConstraints(std::map<int, int> &set_of_constraints)
{
    // construct set of constraints from original received esterel plan
    // example:
    // C : [2:3, 5:6]; node 3 goes after node 2, node 6 goes after node 5

    // delete previous data if any
    set_of_constraints.clear();

    // to print constraints at the end
    std::stringstream ss;

    // init set of temporal constraints (C)
    for(auto eit=original_plan_.edges.begin(); eit!=original_plan_.edges.end(); eit++) {
        // discriminate for interference edges
        if(eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::INTERFERENCE_EDGE ||
            eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::START_END_ACTION_EDGE) {
            // add constraint
            set_of_constraints.insert(std::pair<int, int>(eit->source_ids[0], eit->sink_ids[0]));

            // add constraint to print buffer
            ss << "(" << eit->source_ids[0] << ":" << eit->sink_ids[0] << ")";
            continue;
        }
        // NOTE: condition edges are ignored: we consider only a partially orderes plan
    }

    // print constraints
    ROS_INFO("constraints: %s", ss.str().c_str());
}

bool CSPExecGenerator::checkTemporalConstraints(std::vector<int> &set_of_ordered_nodes,
    std::map<int, int> &set_of_constraints)
{
    // check if set of ordered nodes (F) satisfies the set of constraints (C)

    // iterate over set of constraints
    for(auto cit=set_of_constraints.begin(); cit!=set_of_constraints.end(); cit++) {
        // find key in set_of_ordered_nodes
        std::vector<int>::iterator kit = std::find(set_of_ordered_nodes.begin(),
                            set_of_ordered_nodes.end(), cit->first);
        // check if element was found
        if(kit != set_of_ordered_nodes.end()) {
            // key was found, now find value
            std::vector<int>::iterator vit = std::find(set_of_ordered_nodes.begin(),
                            set_of_ordered_nodes.end(), set_of_constraints.find(cit->first)->second);
            // check if value was found
            if(vit != set_of_ordered_nodes.end())
                // if vit index < kit index, then constraint is violated
                if(std::distance(kit, vit) < 0)
                    return false;
        }
    }

    return true;
}

void CSPExecGenerator::testFunctions()
{
    // example of how to use CheckTemporalConstraints()

//     std::vector<int> set_of_ordered_nodes = {1, 3, 2, 4, 5, 6}; // violates constraints
//     // std::vector<int> set_of_ordered_nodes = {1, 2, 3, 4, 5, 6}; // does not violate constraints
//     std::map<int, int> set_of_constraints = {{2, 3},{5, 6}};
//     if(checkTemporalConstraints(set_of_ordered_nodes, set_of_constraints))
//         ROS_INFO("constraints are satisfied");
//     else
//         ROS_INFO("constraints are violated");
//
//     // test function findNodesBeforeA
//     std::vector<int> open_list = {1, 4, 3, 2};
//     std::vector<int> s = findNodesBeforeA(3, open_list);
//     //print s
//     std::stringstream ss;
//     for(auto nit=s.begin(); nit!=s.end(); nit++) {
//         ss << *nit;
//         ss << ",";
//     }
//     ROS_INFO("nodes before a : {%s}", ss.str().c_str());

    ROS_INFO("Generating execution alternatives test is computing now");

    while(!is_esterel_plan_received_) {
        ROS_INFO("waiting for plan to arrive");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    // lower flag
    is_esterel_plan_received_ = false;

    // this function is put here for testing purposes
    if(generatePlans())
    {
        // indicates that at least one valid execution was found
        ROS_INFO("Found %ld valid execution(s)", ordered_plans_.size());

        // remove, print plans
        for(auto it=ordered_plans_.begin(); it!=ordered_plans_.end(); it++)
            printNodes("plan", *it);
    }
    else
    {
        // indicates that no valid execution was found, means replanning is needed
        ROS_INFO("No valid execution was found, replanning is needed");

        // remove, print plans
        ROS_INFO("An error happened, but anyway I will show you the plans I have found:");
        for(auto it=ordered_plans_.begin(); it!=ordered_plans_.end(); it++)
            printNodes("plan", *it);
    }
}

bool CSPExecGenerator::getAction(int action_id, std::string &action_name, std::vector<std::string> &params,
    rosplan_dispatch_msgs::EsterelPlan &plan, bool &action_start)
{
    // input a node id and return the action name and params

    // delete previous data if any
    params.clear();

    // iterate over the original plan
    for(auto nit=plan.nodes.begin(); nit!=plan.nodes.end(); nit++) {

        // check if node id matches with node
        if(nit->node_id == action_id) {

            // get name, write return value (1) by reference
            action_name = nit->action.name;
            // extract params
            for(auto pit=nit->action.parameters.begin(); pit!=nit->action.parameters.end(); pit++)
                // write return value (2) by reference
                params.push_back(pit->value);

            // discriminate node action start or end
            if(nit->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {
                // write return value (3) by reference
                action_start = true;
                return true;
            }
            else if(nit->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {
                // write return value (3) by reference
                action_start = false;
                return true;
            }
            else {
                ROS_ERROR("node should be either start or end, while getting action (id: %d)", action_id);
                return false;
            }
        }
    }

    ROS_ERROR("get action: node id : %d, was not found in plan", action_id);
    return false;
}

bool CSPExecGenerator::getStartNodeID(int end_node_id, int &action_start_node_id)
{
    // receive as input a node id from a end action node,
    // return by reference the node id of the correspondent action start

    // get the node from the node id
    for(auto nit=original_plan_.nodes.begin(); nit!=original_plan_.nodes.end(); nit++) {
        // find needed node by comparing id
        if(!(nit->node_id == end_node_id))
            continue;

        // id matches, now ensure it is an action end node
        if(!(nit->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END)) {
            ROS_ERROR("Cannot return action start node id, Node id matches but is not an action end");
            return false;
        }

        // node is an action end node, find its correspondent start node id

        // iterate over the edges in id's
        for(auto enit=nit->edges_in.begin(); enit!=nit->edges_in.end(); enit++) {
            // get complete edge msg from edge id
            for(auto eit=original_plan_.edges.begin(); eit!=original_plan_.edges.end(); eit++) {
                // ignore edges which id don't match
                if(eit->edge_id != *enit) {
                    continue;
                }

                // ensure is an "action end" edge
                if(eit->edge_type==rosplan_dispatch_msgs::EsterelPlanEdge::START_END_ACTION_EDGE) {
                    // return by reference the node id of the start action
                    action_start_node_id = eit->source_ids[0];
                    ROS_INFO("found correspondent action start node (%d) from action end node (%d)", action_start_node_id, end_node_id);
                    return true;
                }
            }
        }
    }

    ROS_ERROR("failed to get action start node id (of action end :%d)", end_node_id);
    return false;
}

bool CSPExecGenerator::validNodes(std::vector<int> &open_list, std::vector<int> &valid_nodes)
{
    // iterate over open list (O), check if node preconditions are met in current state (S)

    // ensure open list is not empty
    if(!open_list.size() > 0) {
        ROS_INFO("open list is empty, while checking validNodes");
        return false;
    }

    // iterate over open list
    for(auto nit=open_list.begin(); nit!=open_list.end(); nit++) {

        // get action properties (name, params, type) from node id
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        if(!getAction(*nit, action_name, params, original_plan_, action_start)) {
            ROS_ERROR("failed to get action properties (while getting valid nodes)");
            return false;
        }

        if(action_start) {
            // check if action start + overall preconditions are met
            ROS_INFO("check if action start (id: %d) is applicable : (%s)", *nit,
                         action_simulator_.convertPredToString(action_name, params).c_str());
            if(action_simulator_.isActionStartAplicable(action_name, params)) {
                if(action_simulator_.isActionOverAllAplicable(action_name, params)) {
                    ROS_INFO("(action start) node is valid (id: %d), add to valid list", *nit);
                    // node is valid, add to list
                    valid_nodes.push_back(*nit);
                }
                else
                    ROS_INFO("(action start) node %d is NOT valid", *nit);
            }
        }
        else {
            // check if action end + overall preconditions are met
            ROS_INFO("check if action end (id: %d) is applicable : (%s)", *nit,
                         action_simulator_.convertPredToString(action_name, params).c_str());
            if(action_simulator_.isActionEndAplicable(action_name, params)) {
                if(action_simulator_.isActionOverAllAplicable(action_name, params)) {
                    ROS_INFO("(action end) node is valid, check if correspondent action start is ordered");

                    // Ignore action ends in validNodes for actions that have not started

                    // get action id of start node
                    int start_node_id;
                    if(!getStartNodeID(*nit, start_node_id))
                        return false;

                    // add only if start node is already ordered
                    bool ordered = false;
                    for(auto onit=ordered_nodes_.begin(); onit!=ordered_nodes_.end(); onit++)
                        if(start_node_id == *onit) {
                            ordered = true;
                            // node is valid, add to list
                            valid_nodes.push_back(*nit);

                            // remove
                            ROS_INFO("checked if correspondent action start is ordered : yes is ordered, add action (%d) to valid list", *nit);
                        }

                    if(!ordered)
                        ROS_INFO("skipping applicable action end (%d) because action start (%d) is not ordered yet", *nit, start_node_id);

                    // printNodes("ordered nodes F", ordered_nodes_);
                }
            }
        }
    }

    // print valid nodes
    std::stringstream ss;
    if(valid_nodes.size() > 0) {
        printNodes("valid nodes", valid_nodes);
        return true;
    }
    else
        ROS_INFO("no valid nodes were found");

    // no valid nodes
    return false;
}

std::vector<int> CSPExecGenerator::findNodesBeforeA(int a, std::vector<int> &open_list)
{
    // find all nodes b in open list (O) which ordering constraints enforce them before a

    std::vector<int> nodes_before_a;

    // make sure constraints are non empty
    if(set_of_constraints_.size() > 0) {
        // iterate over the constraints
        for(auto cit=set_of_constraints_.begin(); cit!=set_of_constraints_.end(); cit++) {
            if(cit->second == a) {
                // relevant constraint, find correspondent node in open list
                for(auto oit=open_list.begin(); oit!=open_list.end() ;oit++) {
                    if(*oit == cit->first)
                        // found constraint in open list which should happen before a, aka skipped node
                        // this happens when e.g. a human helps the robot to do the action
                        nodes_before_a.push_back(cit->first);
                }
            }
        }
    }

    return nodes_before_a;
}

bool CSPExecGenerator::orderNodes(std::vector<int> open_list)
{
    // shift nodes from open list (O) to ordered plans (R)
    // offering all possible different execution alternatives via DFS (Depth first search)

    ROS_INFO("order nodes (recurse)");

    if(!checkTemporalConstraints(ordered_nodes_, set_of_constraints_)) {
        ROS_ERROR("temporal constraints not satisfied");
        return false;
    }

    // check if goals are achieved
    ROS_INFO("checking if goals are achieved...");
    if(action_simulator_.areGoalsAchieved()) {
        ROS_WARN("goals achieved! valid plan found as follows:"); // TODO: move back to ROS_INFO
        printNodes("plan", ordered_nodes_);

        // add new valid ordering to ordered plans (R)
        ordered_plans_.push_back(ordered_nodes_);

        // backtrack: popf, remove last element from f, store in variable and revert that action
        ROS_INFO("backtrack because goal was achieved");
        if(!ordered_nodes_.empty()) { // ensure stack is not empty
            // revert action
            ROS_INFO("poping action (removing from stack), reverting action to S, action id: %d", ordered_nodes_.back());
            std::string action_name;
            std::vector<std::string> params;
            bool action_start;
            if(getAction(ordered_nodes_.back(), action_name, params, original_plan_, action_start)) {
                // remove
                ROS_INFO("KB after reverting action %d", ordered_nodes_.back());

                ordered_nodes_.pop_back(); // eliminate last node from stack
                // getAction() finds out if last element of "f" is action start or end, info is in action_start boolean
                if(action_start)
                    action_simulator_.revertActionStart(action_name, params);
                else
                    action_simulator_.revertActionEnd(action_name, params);

                // remove
                action_simulator_.printInternalKBFacts();
            }
            else
                ROS_ERROR("failed to get action properties (while backtracking because goal was achieved)");
        }

        return true;
    }
    else
        ROS_INFO("goals not achieved yet");

    // see which nodes preconditions are met and construct valid nodes list (V)
    // remove
    printNodes("open list", open_list); // print open list for debuggin purposes
    ROS_INFO("finding valid nodes from open list now");
    std::vector<int> valid_nodes;
    validNodes(open_list, valid_nodes);
    if(valid_nodes.size() == 0) {
        ROS_ERROR("valid nodes are empty");

        // backtrack: popf, remove last element from f, store in variable and revert that action
        ROS_INFO("backtrack because nodes are empty");
        if(!ordered_nodes_.empty()) { // ensure stack is not empty
            // revert action
            ROS_INFO("poping action (removing from stack), reverting action to S, action id: %d", ordered_nodes_.back());
            std::string action_name;
            std::vector<std::string> params;
            bool action_start;
            if(getAction(ordered_nodes_.back(), action_name, params, original_plan_, action_start)) {
                // remove
                ROS_INFO("KB after reverting action %d", ordered_nodes_.back());

                ordered_nodes_.pop_back(); // eliminate last node from stack
                // getAction() finds out if last element of "f" is action start or end, info is in action_start boolean
                if(action_start)
                    action_simulator_.revertActionStart(action_name, params);
                else
                    action_simulator_.revertActionEnd(action_name, params);

                // remove
                action_simulator_.printInternalKBFacts();
            }
            else
                ROS_ERROR("failed to get action properties (while backtracking, valid nodes empty)");
        }

        return false;
    }
    else
        ROS_INFO("valid nodes search has finished: found valid nodes");

    // remove , keep track of the branching factor
    branching_factor_.push_back(valid_nodes.size());

    // iterate over actions in valid nodes (V)
    for(auto a=valid_nodes.begin(); a!=valid_nodes.end(); a++) {

        // find all nodes (b) ordered before (a), s = skipped nodes
        std::vector<int> s = findNodesBeforeA(*a, open_list);

        // remove
        ROS_INFO("add action to stack : %d", *a);
        printNodes("stack before adding", ordered_nodes_);

        // order a, (add to queue)
        ordered_nodes_.push_back(*a);

        // remove
        printNodes("stack after adding", ordered_nodes_);

        // remove
        ROS_INFO("remove action and skipped actions from open list");

        // remove a (action) and s (skipped nodes) from open list (O)
        std::vector<int> open_list_copy = open_list;
        if(open_list_copy.size() > 0) { // make sure open list is not empty
            printNodes("open list", open_list_copy);
            // find current action "a" in open list and get a pointer to it
            std::vector<int>::iterator ait = std::find(open_list_copy.begin(),open_list_copy.end(), *a);
            open_list_copy.erase(ait); // delete current action "a"
            // iterate over s (skipped nodes) and delete one at a time
            if(s.size() > 0)
                for(auto sit=s.begin(); sit!=s.end(); sit++) {
                    // find and remove elements of s
                    std::vector<int>::iterator sp = std::find(open_list_copy.begin(),open_list_copy.end(), *sit);
                    open_list_copy.erase(sp);
                }
        }
        else {
            // put here to prevent seg fault error, but if code reaches here then it would be a bug
            ROS_ERROR("open list is empty, this means an implementation error");
            return false;
        }

        // remove
        ROS_INFO("apply action : (%d), to current state S", *a);

        // apply action a to current state (S)

        // get action properties (name, params, type) from node id
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        if(!getAction(*a, action_name, params, original_plan_, action_start)) {
            ROS_ERROR("failed to get action properties (while applying action)");
            return false;
        }

        // remove
        ROS_INFO("KB before applying action %d", *a);
        action_simulator_.printInternalKBFacts();

        if(action_start) {
            // action start
            ROS_INFO("apply action a : (%s)", action_simulator_.convertPredToString(action_name, params).c_str());
            if(!action_simulator_.simulateActionStart(action_name, params)) {
                ROS_ERROR("could not simulate action start");
                return false;
            }
        }
        else {
            // action end
            if(!action_simulator_.simulateActionEnd(action_name, params)) {
                ROS_ERROR("could not simulate action end");
                return false;
            }
        }

        // remove
        ROS_INFO("KB after applying action %d", *a);
        action_simulator_.printInternalKBFacts();

        // recurse
        orderNodes(open_list_copy);
    }

    // backtrack: popf, remove last element from f, store in variable and revert that action
    ROS_INFO("backtrack because for loop ended (valid nodes exhausted)");
    printNodes("branching factor", branching_factor_);
    if(!ordered_nodes_.empty()) { // ensure stack is not empty
        // revert action
        ROS_INFO("poping action (removing from stack), reverting action to S, action id: %d", ordered_nodes_.back());
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        if(getAction(ordered_nodes_.back(), action_name, params, original_plan_, action_start)) {
            // remove
            ROS_INFO("KB after reverting action %d", ordered_nodes_.back());

            ordered_nodes_.pop_back(); // eliminate last node from stack
            // getAction() finds out if last element of "f" is action start or end, info is in action_start boolean
            if(action_start)
                action_simulator_.revertActionStart(action_name, params);
            else
                action_simulator_.revertActionEnd(action_name, params);

            // remove
            action_simulator_.printInternalKBFacts();
        }
        else
            ROS_ERROR("failed to get action properties (while backtracking, nodes exhausted, end of for loop)");
    }

    return true;
}

bool CSPExecGenerator::generatePlans()
{
    // get current state (S) and store in memory
    action_simulator_.saveKBSnapshot();

    // init open list (O), initially contains all nodes in partial order plan
    std::vector<int> open_list;
    for(auto nit=original_plan_.nodes.begin(); nit!=original_plan_.nodes.end(); nit++) { // nit=node iterator
        // do not add plan start node
        if(nit->node_type != rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START)
            open_list.push_back(nit->node_id);
    }

    // init set of constraints (C)
    initConstraints(set_of_constraints_);

    // init set of ordered nodes (F)
    ordered_nodes_.clear();

    // init set of totally ordered plans (R)
    ordered_plans_.clear();

    // if true, it means at least one valid execution alternative was found
    ROS_INFO("Finding all possible executions now");
    return orderNodes(open_list);
}

bool CSPExecGenerator::srvCB(rosplan_dispatch_msgs::ExecAlternatives::Request& req, rosplan_dispatch_msgs::ExecAlternatives::Response& res)
{
    ROS_INFO("generating execution alternatives service is computing now");

    if(!is_esterel_plan_received_) {
        ROS_ERROR("requires esterel plan input has not being received yet, can't generate exec alternatives");
        // set result of srv as failure
        res.replan_needed = true;
        res.exec_alternatives_generated = false;
        return true;
    }

    // lower flag
    is_esterel_plan_received_ = false;

    if(generatePlans())
    {
        // indicates that at least one valid execution was found
        res.replan_needed = false;
        res.exec_alternatives_generated = true;
        ROS_INFO("Found %ld valid execution(s)", ordered_plans_.size());

        // remove, print plans
        for(auto it=ordered_plans_.begin(); it!=ordered_plans_.end(); it++)
            printNodes("plan", *it);
    }
    else
    {
        // indicates that no valid execution was found, means replanning is needed
        res.replan_needed = true;
        res.exec_alternatives_generated = false;
        ROS_INFO("No valid execution was found, replanning is needed");

        // remove, print plans
        ROS_INFO("An error happened, but anyway I will show you the plans I have found:");
        for(auto it=ordered_plans_.begin(); it!=ordered_plans_.end(); it++)
            printNodes("plan", *it);
    }

    ROS_INFO("Generating execution alternatives service has finished");

    return true;
}

void CSPExecGenerator::update()
{
    // check if callbacks have requests
    ros::spinOnce();
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "csp_exec_generator_node");
    ROS_INFO("node is going to initialize...");

    // create object of the node class (CSPExecGenerator)
    CSPExecGenerator csp_exec_generator_node;

    // setup node frequency
    double node_frequency = 1.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 1.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");

    csp_exec_generator_node.testFunctions();

//     while (ros::ok())
//     {
//         // main loop function
//         csp_exec_generator_node.update();
//
//         // sleep to control the node frequency
//         loop_rate.sleep();
//     }

    return 0;
}
