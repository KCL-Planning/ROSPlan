/*
 * Copyright [2019] <KCL - ISR collaboration>
 *
 * KCL: King's College London
 * ISR: Institue for Systems and Robotics
 *
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk), Oscar Lima (olima_84@yahoo.com)
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

    if(generateFullyConnectedPlan())
        // indicates that at least one valid execution was found
        ROS_INFO("Found valid execution(s)");
    else
        // indicates that no valid execution was found, means replanning is needed
        ROS_INFO("No valid execution was found, replanning is needed");
}

bool CSPExecGenerator::getAction(int action_id, std::string &action_name, std::vector<std::string> &params,
    rosplan_dispatch_msgs::EsterelPlan &plan)
{
    // input a node id and return the action name and params

    // delete previous data if any
    params.clear();

    // iterate over the original plan
    for(auto nit=plan.nodes.begin(); nit!=plan.nodes.end(); nit++) {
        // check if node id matches with node
        if(nit->node_id == action_id) {
            action_name = nit->action.name;
            // extract params
            for(auto pit=nit->action.parameters.begin(); pit!=nit->action.parameters.end(); pit++)
                params.push_back(pit->value);

            // discriminate node action start or end
            if(nit->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START)
                return true;
            else if(nit->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END)
                return false;
            else {
                ROS_ERROR("node should be either start or end, while getting action");
                return false;
            }
        }
    }

    ROS_ERROR("get action: node id was not found in plan");
    return false;
}

bool CSPExecGenerator::validNodes(std::vector<int> &open_list, std::vector<int> &valid_nodes)
{
    // iterate over open list (O), check if node preconditions are met in current state (P)

    for(auto nit=open_list.begin(); nit!=open_list.end(); nit++) {
        std::string action_name;
        std::vector<std::string> params;
        bool action_start = getAction(*nit, action_name, params, original_plan_);
        if(action_start) {
            ROS_DEBUG("check if action start is applicable : (%s)",
                         action_simulator_.convertPredToString(action_name, params).c_str());
            if(action_simulator_.isActionStartAplicable(action_name, params)) {
                valid_nodes.push_back(*nit);
            }
        }
        else {
            ROS_DEBUG("check if action end is applicable : (%s)",
                         action_simulator_.convertPredToString(action_name, params).c_str());
            if(action_simulator_.isActionEndAplicable(action_name, params)) {
                valid_nodes.push_back(*nit);
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
    // find all nodes b in open list (O) ordered before a

    std::vector<int> nodes_before_a;

    // iterate over all node b in open list (O), if b < a then add to s, return s
    for(auto b=open_list.begin(); b!=open_list.end(); b++) {
        // find a and b in open list
        std::vector<int>::iterator ait = std::find(open_list.begin(), open_list.end(), a);
        std::vector<int>::iterator bit = std::find(open_list.begin(), open_list.end(), *b);

        if(std::distance(ait, bit) < 0)
            nodes_before_a.push_back(*b);
    }

    return nodes_before_a;
}

bool CSPExecGenerator::orderNodes()
{
    ROS_INFO("ordering nodes now");

    // shift nodes from open list (O) to ordered plans (R) offering different execution alternatives
    if(!checkTemporalConstraints(ordered_nodes_, set_of_constraints_)) {
        ROS_ERROR("temporal constraints not satisfied");
        return false;
    }

    // check if goals are achieved
    if(action_simulator_.areGoalsAchieved()) {
        ROS_INFO("valid plan found as follows:");
        printNodes("plan", ordered_nodes_);

        // add new valid ordering to ordered plans (R)
        ordered_plans_.push_back(ordered_nodes_);

        return true;
    }

    // see which nodes preconditions are met and construct valid nodes list (V)
    std::vector<int> valid_nodes;
    validNodes(open_list_, valid_nodes);
    if(valid_nodes.size() == 0) {
        ROS_ERROR("valid nodes are empty");
        return false;
    }

    // iterate over actions in valid nodes (V)
    for(auto a=valid_nodes.begin(); a!=valid_nodes.end(); a++) {
        // find all nodes (b) ordered before (a)
        std::vector<int> s = findNodesBeforeA(*a, open_list_);

        // remove, print a and nodes before a
        ROS_INFO("a=%d", *a);
        printNodes("open list=", open_list_);
        printNodes("Nodes before a in open list (O), s=", s);

        // order a
        ordered_nodes_.push_back(*a);

        //remove
        ROS_INFO("remove a and s from open list");
        printNodes("open list before removing", open_list_);

        // remove a and s from open list (O)
        std::vector<int>::iterator ait = std::find(open_list_.begin(),open_list_.end(), *a);
        open_list_.erase(ait);
        // iterate over s
        if(s.size() > 0)
            for(auto sit=s.begin(); sit!=s.end(); sit++) {
                // find and remove elements of s
                std::vector<int>::iterator sp = std::find(open_list_.begin(),open_list_.end(), *sit);
                open_list_.erase(sp);
            }

        // remove
        printNodes("open list after removing", open_list_);
        ROS_INFO("applying action a, world state before:");
        action_simulator_.printInternalKBFacts();

        // apply action a to current state (P)
        std::string action_name;
        std::vector<std::string> params;
        if(getAction(*a, action_name, params, original_plan_)) {
            // action start
            ROS_INFO("applying action a (start), world state after:");
            if(!action_simulator_.simulateActionStart(action_name, params)) {
                ROS_ERROR("could not simulate action start");
                return false;
            }
        }
        else {
            // action end
            ROS_INFO("applying action a (end), world state after:");
            if(!action_simulator_.simulateActionEnd(action_name, params)) {
                ROS_ERROR("could not simulate action end");
                return false;
            }
        }

        // remove
        action_simulator_.printInternalKBFacts();

        // recurse
        return orderNodes();
    }

    // should never reach this point
    ROS_ERROR("Ran out of nodes without achieving goal");
    return false;
}

bool CSPExecGenerator::generateFullyConnectedPlan()
{
    // get current state (P) and store in memory
    action_simulator_.saveKBSnapshot();

    // init open list (O), initially contains all nodes in partial order plan
    open_list_.clear();
    for(auto nit=original_plan_.nodes.begin(); nit!=original_plan_.nodes.end(); nit++) { // nit=node iterator
        // do not add plan start node
        if(nit->node_type != rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START)
            open_list_.push_back(nit->node_id);
    }

    // init set of constraints (C)
    initConstraints(set_of_constraints_);

    // init set of ordered nodes (F)
    ordered_nodes_.clear();

    // init set of totally ordered plans (R)
    ordered_plans_.clear();

    // if true, it means at least one valid execution alternative was found
    return orderNodes();
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

    if(generateFullyConnectedPlan())
    {
        // indicates that at least one valid execution was found
        res.replan_needed = false;
        res.exec_alternatives_generated = true;
        ROS_INFO("Found valid execution(s)");
    }
    else
    {
        // indicates that no valid execution was found, means replanning is needed
        res.replan_needed = true;
        res.exec_alternatives_generated = false;
        ROS_INFO("No valid execution was found, replanning is needed");
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
