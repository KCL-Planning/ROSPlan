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

    // publications, executions alternatives (multiple esterel plans)
    pub_valid_plans_ = nh_.advertise<rosplan_dispatch_msgs::EsterelPlanArray>("valid_plans", 1);

    // remove, publish test esterel plan
    pub_esterel_plan_ = nh_.advertise<rosplan_dispatch_msgs::EsterelPlan>("partial_order_plan", 1);

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

    // remove conditional edges from plan, store them in member variable partial_order_plan_
    partial_order_plan_ = removeConditionalEdges(original_plan_);

    // raise flag to indicate a msg has been received in callback
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
    ROS_DEBUG("constraints: %s", ss.str().c_str());
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
    // set here what you want to test
    bool check_multiple_plan_generation = true;
    bool test_temporal_constraints = false;

    if(test_temporal_constraints) {
        // example of how to use CheckTemporalConstraints()

        ROS_INFO("testing temporal constraints now");
        std::vector<int> set_of_ordered_nodes = {1, 3, 2, 4, 5, 6}; // violates constraints
        // std::vector<int> set_of_ordered_nodes = {1, 2, 3, 4, 5, 6}; // does not violate constraints
        std::map<int, int> set_of_constraints = {{2, 3},{5, 6}};
        if(checkTemporalConstraints(set_of_ordered_nodes, set_of_constraints))
            ROS_DEBUG("constraints are satisfied");
        else
            ROS_DEBUG("constraints are violated");

        // test function findNodesBeforeA
        std::vector<int> open_list = {1, 4, 3, 2};
        std::vector<int> s = findNodesBeforeA(3, open_list);
        //print s
        std::stringstream ss;
        for(auto nit=s.begin(); nit!=s.end(); nit++) {
            ss << *nit;
            ss << ",";
        }
        ROS_DEBUG("nodes before a : {%s}", ss.str().c_str());
    }

    if(check_multiple_plan_generation) {
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

            for(auto it=ordered_plans_.begin(); it!=ordered_plans_.end(); it++)
            printNodes("plan", *it);
        }
        else
        {
            // error while computing the multiple possible executions
            ROS_INFO("No valid execution was found, replanning is needed");

            // maybe some plans were able to compute during the search exploration...
            ROS_INFO("An error happened, but anyway I will show you the plans I have found:");
            for(auto it=ordered_plans_.begin(); it!=ordered_plans_.end(); it++)
            printNodes("plan", *it);
        }
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
                    ROS_DEBUG("found correspondent action start node (%d) from action end node (%d)", action_start_node_id, end_node_id);
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
        ROS_DEBUG("open list is empty, while checking validNodes");
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
            ROS_DEBUG("check if action start (id: %d) is applicable : (%s)", *nit,
                         action_simulator_.convertPredToString(action_name, params).c_str());
            if(action_simulator_.isActionStartApplicable(action_name, params)) {
                ROS_DEBUG("(action start) node is valid (id: %d), add to valid list", *nit);
                // node is valid, add to list
                valid_nodes.push_back(*nit);
            }
            else
                ROS_DEBUG("(action start) node %d is NOT valid", *nit);
        }
        else {
            // check if action end + overall preconditions are met
            ROS_DEBUG("check if action end (id: %d) is applicable : (%s)", *nit,
                         action_simulator_.convertPredToString(action_name, params).c_str());
            if(action_simulator_.isActionEndApplicable(action_name, params)) {
                ROS_DEBUG("(action end) node is valid, check if correspondent action start is ordered");

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

                        ROS_DEBUG("checked if correspondent action start is ordered : yes is ordered, add action (%d) to valid list", *nit);
                    }

                if(!ordered)
                    ROS_DEBUG("skipping applicable action end (%d) because action start (%d) is not ordered yet", *nit, start_node_id);

                // printNodes("ordered nodes F", ordered_nodes_);
            }
        }
    }

    // print valid nodes
    std::stringstream ss;
    if(valid_nodes.size() > 0) {
        //printNodes("valid nodes", valid_nodes);
        return true;
    }
    else
        ROS_DEBUG("no valid nodes were found");

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

void CSPExecGenerator::backtrack(std::string reason_for_backtrack)
{
    // backtrack: popf, remove last element from f, store in variable and revert that action
    ROS_DEBUG("backtrack because %s", reason_for_backtrack.c_str());
    if(!ordered_nodes_.empty()) { // ensure stack is not empty
        // revert action
        ROS_DEBUG("poping action (removing from stack), reverting action to S, action id: %d", ordered_nodes_.back());
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        if(getAction(ordered_nodes_.back(), action_name, params, original_plan_, action_start)) {
            ROS_DEBUG("KB after reverting action %d", ordered_nodes_.back());

            ordered_nodes_.pop_back(); // eliminate last node from stack
            // getAction() finds out if last element of "f" is action start or end, info is in action_start boolean
            if(action_start)
                action_simulator_.revertActionStart(action_name, params);
            else
                action_simulator_.revertActionEnd(action_name, params);

            // action_simulator_.printInternalKBFacts();
        }
        else
            ROS_ERROR("failed to get action properties (while backtracking)");
    }
}

bool CSPExecGenerator::orderNodes(std::vector<int> open_list)
{
    // shift nodes from open list (O) to ordered plans (R)
    // offering all possible different execution alternatives via DFS (Depth first search)

    ROS_DEBUG("order nodes (recurse)");

    if(!checkTemporalConstraints(ordered_nodes_, set_of_constraints_)) {
        ROS_ERROR("temporal constraints not satisfied");
        return false;
    }

    // check if goals are achieved
    ROS_DEBUG("checking if goals are achieved...");
    if(action_simulator_.areGoalsAchieved()) {
        // we print all plans at the end, so only we print here in debug mode
        ROS_DEBUG("goals achieved! valid plan found as follows:");
        // printNodes("plan", ordered_nodes_);

        // add new valid ordering to ordered plans (R)
        ordered_plans_.push_back(ordered_nodes_);

        // backtrack: popf, remove last element from f, store in variable and revert that action
        backtrack("goal was achieved");
        return true;
    }
    else
        ROS_DEBUG("goals not achieved yet");

    // see which nodes preconditions are met and construct valid nodes list (V)
    // printNodes("open list", open_list); // print open list for debuggin purposes
    ROS_DEBUG("finding valid nodes from open list now");
    std::vector<int> valid_nodes;
    validNodes(open_list, valid_nodes);
    if(valid_nodes.size() == 0) {
        ROS_DEBUG("valid nodes are empty");

        // backtrack: popf, remove last element from f, store in variable and revert that action
        backtrack("nodes are empty");
        return false;
    }
    else
        ROS_DEBUG("valid nodes search has finished: found valid nodes");

    // iterate over actions in valid nodes (V)
    for(auto a=valid_nodes.begin(); a!=valid_nodes.end(); a++) {

        // find all nodes (b) ordered before (a), s = skipped nodes
        std::vector<int> s = findNodesBeforeA(*a, open_list);

        ROS_DEBUG("add action to stack : %d", *a);
        // printNodes("stack before adding", ordered_nodes_);

        // order a, (add to queue)
        ordered_nodes_.push_back(*a);

        // printNodes("stack after adding", ordered_nodes_);

        ROS_DEBUG("remove action and skipped actions from open list");

        // remove a (action) and s (skipped nodes) from open list (O)
        std::vector<int> open_list_copy = open_list;
        if(open_list_copy.size() > 0) { // make sure open list is not empty
            // printNodes("open list", open_list_copy);
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

        ROS_DEBUG("apply action : (%d), to current state S", *a);

        // apply action a to current state (S)

        // get action properties (name, params, type) from node id
        std::string action_name;
        std::vector<std::string> params;
        bool action_start;
        if(!getAction(*a, action_name, params, original_plan_, action_start)) {
            ROS_ERROR("failed to get action properties (while applying action)");
            return false;
        }

        ROS_DEBUG("KB before applying action %d", *a);
        action_simulator_.printInternalKBFacts();

        if(action_start) {
            // action start
            ROS_DEBUG("apply action a : (%s)", action_simulator_.convertPredToString(action_name, params).c_str());
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

        // ROS_DEBUG("KB after applying action %d", *a);
        // action_simulator_.printInternalKBFacts();

        // recurse
        orderNodes(open_list_copy);
    }

    // pop last element from stack (ordered_nodes_) revert action
    backtrack("for loop ended (valid nodes exhausted)");
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
        if(nit->node_type != rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START) {
            // remove nodes which are currently/done executing from open list, you receive
            // this information from the dispatcher inside the service call request
            if(std::find(action_executing_.begin(), action_executing_.end(), nit->node_id) != action_executing_.end()) {
                // node found in (executing/done) list
                ROS_INFO("ignoring node (%d) because is currently being/done executed", nit->node_id);
            } else {
                // node not found in (executing/done) list
                open_list.push_back(nit->node_id);
            }
        }
    }

    printNodes("open list", open_list);

    // init set of constraints (C)
    initConstraints(set_of_constraints_);

    // init set of ordered nodes (F)
    ordered_nodes_.clear();

    // init set of totally ordered plans (R)
    ordered_plans_.clear();

    // if true, it means at least one valid execution alternative was found
    return orderNodes(open_list);
}

rosplan_dispatch_msgs::EsterelPlan CSPExecGenerator::removeConditionalEdges(rosplan_dispatch_msgs::EsterelPlan &esterel_plan)
{
    // remove all conditional edges in a esterel plan, plan is received and modified by reference

    rosplan_dispatch_msgs::EsterelPlan output_plan;

    // iterate over originally reecived esterel_plan and copy all nodes
    for(auto nit=esterel_plan.nodes.begin(); nit!=esterel_plan.nodes.end(); nit++) {
        output_plan.nodes.push_back(*nit);
    }

    // iterate over the edges, copy all but leave the conditional ones out
    for(auto eit=esterel_plan.edges.begin(); eit!=esterel_plan.edges.end(); eit++) {
        if(eit->edge_type != rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE) {
            // add edge
            output_plan.edges.push_back(*eit);
        }
    }

    // remove
    for(auto eit=output_plan.edges.begin(); eit!=output_plan.edges.end(); eit++) {
        if(eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE) {
            // remove
            ROS_INFO("did not worked!");
        }
    }

    return output_plan;
}

rosplan_dispatch_msgs::EsterelPlan CSPExecGenerator::convertListToEsterel(std::vector<int> &ordered_nodes)
{
    // add constrains to the partially ordered plan (esterel plan without conditional edges)

    rosplan_dispatch_msgs::EsterelPlan esterel_plan = partial_order_plan_;

    // NOTE: it is assumed that input plan already does not has conditional edges and is therefore a partial order plan
    // add edges to partial_order_plan_ (aka esterel_plan), we will add them as conditional but it is a hack, they are not conditional edges

    rosplan_dispatch_msgs::EsterelPlanEdge last_edge = esterel_plan.edges.back();
    int edge_id_count = last_edge.edge_id;

    // iterate over the ordered nodes
    for(auto nit=ordered_nodes.begin(); nit!=(ordered_nodes.end() - 1); nit++) {
        // assume each pair of nodes is a constraint
        rosplan_dispatch_msgs::EsterelPlanEdge edge_msg;

        // fill edge msg
        edge_msg.edge_type = rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE;
        edge_msg.edge_id = ++edge_id_count;
        std::string string = "edge_" + std::to_string(edge_id_count);
        edge_msg.edge_name = string;
        edge_msg.signal_type = 0;
        edge_msg.source_ids.push_back(*nit);
        edge_msg.sink_ids.push_back(*(nit + 1));
        edge_msg.duration_lower_bound = 0.0;
        edge_msg.duration_upper_bound = 0.0;

        // add edge (as conditional edge -> workaround)
        esterel_plan.edges.push_back(edge_msg);
    }

    return esterel_plan;
}

bool CSPExecGenerator::srvCB(rosplan_dispatch_msgs::ExecAlternatives::Request& req, rosplan_dispatch_msgs::ExecAlternatives::Response& res)
{
    ROS_INFO("generating execution alternatives service is computing now");

    if(!is_esterel_plan_received_) {
        // esterel plan not received yet!
        ROS_ERROR("Generation of plan alternatives requires an esterel plan as input but it has not being received yet");

        // replanning is needed, to enforce reveiving the esterel plan
        res.replan_needed = true;

        // indicate that no valid execution was found
        res.exec_alternatives_generated = false;

        // service call was succesful (regardless if at least one plan was found or not)
        return true;
    }

    // lower flag to force the node to receive a new plan if a new request comes
    // is_esterel_plan_received_ = false;

    // save nodes which are being/done executing in member variable to be removed from open list (skipped)
    action_executing_ = req.actions_executing;

    if(generatePlans()) // compute exec alternatives
    {
        // indicates that at least one valid execution was found
        res.replan_needed = false;
        res.exec_alternatives_generated = true;
        ROS_INFO("Found %ld valid execution(s)", ordered_plans_.size());

        // print plans for debugging purposes
        for(auto it=ordered_plans_.begin(); it!=ordered_plans_.end(); it++)
            printNodes("plan", *it);

        // transform ordered list into esterel plans
        // iterate over valid plans convert to esterel plan and append to estereal array msg
        rosplan_dispatch_msgs::EsterelPlan esterel_plan_msg;
        rosplan_dispatch_msgs::EsterelPlanArray exec_aternatives_msg;
        for(auto pit=ordered_plans_.begin(); pit!=ordered_plans_.end(); pit++) {
            // convert list of orderes nodes into esterel plan (reuses the originally received esterel plan)
            esterel_plan_msg = convertListToEsterel(*pit);

            // append to esterel array
            exec_aternatives_msg.esterel_plans.push_back(esterel_plan_msg);
            // probabilities missing, for now we fill same prob
            double plan_success_probability = 1.0;
            exec_aternatives_msg.plan_success_prob.push_back(plan_success_probability);
        }

        // publish esterel array msg
        pub_valid_plans_.publish(exec_aternatives_msg);
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
    ROS_DEBUG("node is going to initialize...");

    // create object of the node class (CSPExecGenerator)
    CSPExecGenerator csp_exec_generator_node;

    // setup node frequency
    double node_frequency = 0.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 10.0);
    ROS_DEBUG("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_DEBUG("Node initialized.");

    // useful for debugging without having to call service, uncomment if needed
    // csp_exec_generator_node.testFunctions();

    while (ros::ok())
    {
        // main loop function
        csp_exec_generator_node.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}
