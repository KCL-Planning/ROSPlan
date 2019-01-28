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
    sub_esterel_plan_ = nh_.subscribe("/rosplan_parsing_interface/complete_plan2", 1, &CSPExecGenerator::esterelPlanCB, this);

    // publications
    // array of esterel plans with different execution alternatives
    pub_set_of_solutions_ = nh_.advertise<rosplan_dispatch_msgs::EsterelPlanArray>("set_of_solutions", 1);
    // partially ordered plan = original plan - conditional edges = plan with only interference edges
    pub_pop_ = nh_.advertise<rosplan_dispatch_msgs::EsterelPlan>("/rosplan_parsing_interface/complete_plan", 1);

    // services: compute different execution alternatives from a partially ordered esterel plan (a plan
    // with no conditional edges, but only interference edges)
    srv_gen_alternatives_ = nh_.advertiseService("gen_exec_alternatives", &CSPExecGenerator::srvCB, this);

    // call constructor, mirror KB (query real KB and get its data) but without facts and goals
    action_simulator_ = new ActionSimulator(true, false);
}

CSPExecGenerator::~CSPExecGenerator()
{
    // delete pointer
    delete action_simulator_;

    // shut down publishers and subscribers
    sub_esterel_plan_.shutdown();
    pub_set_of_solutions_.shutdown();
    pub_pop_.shutdown();
}

void CSPExecGenerator::esterelPlanCB(const rosplan_dispatch_msgs::EsterelPlan::ConstPtr& msg)
{
    original_plan_ = *msg;
    is_esterel_plan_received_ = true;
}

void CSPExecGenerator::initConstraints(std::map<int, int> &set_of_constraints)
{
    // construct set of contraints from original received esterel plan
    // example:
    // C : [2:3, 5:6]; node 3 goes after node 2, node 6 goes after node 5

    // delete previous data if any
    set_of_constraints.clear();

    // init set of temporal constraints (C)
    for(auto eit=original_plan_.edges.begin(); eit!=original_plan_.edges.begin(); eit++) { // eit = edge iterator
        // discriminate for interference edges
        if(eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::INTERFERENCE_EDGE ||
            eit->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::START_END_ACTION_EDGE) {
            // add constraint
            set_of_constraints.insert(std::pair<int, int>(eit->source_ids[0], eit->sink_ids[0]));
            continue;
        }
        // NOTE: condition edges are ignored: we consider only a partially orderes plan
    }
}

bool CSPExecGenerator::checkTemporalConstraints(std::vector<int> &set_of_ordered_nodes,
    std::map<int, int> &set_of_constraints)
{
    // check if set of ordered nodes (F) satisfies the set of constraints (C)

    // TODO

    return true;
}

bool CSPExecGenerator::areGoalsAchieved(ActionSimulator &as)
{
    // check if curent state (P) satisfies all goals proposed in (G)

    if(as.areGoalsAchieved()) {
        // add new valid ordering to ordered plans (R)
        // TODO

        // return success
        return true;
    }

    // goals are not achieved yet
    return false;
}

std::vector<rosplan_dispatch_msgs::EsterelPlanNode> CSPExecGenerator::validNodes(
    std::vector<rosplan_dispatch_msgs::EsterelPlanNode> open_list)
{
    // iterate over open list (O), check if node preconditions are met in current state (P)

    std::vector<rosplan_dispatch_msgs::EsterelPlanNode> valid_nodes;

    // TODO: complete
    rosplan_dispatch_msgs::EsterelPlanNode node;
    valid_nodes.push_back(node);

    return valid_nodes;
}

bool CSPExecGenerator::orderNodes()
{
    // shift nodes from open list (O) to ordered plans (R) offering different execution alternatives
    if(!checkTemporalConstraints(ordered_nodes_, set_of_constraints_))
        return false;

    // TODO: largely unfinished

    return true;
}

bool CSPExecGenerator::generateFullyConnectedPlan()
{
    // get current state (P) and store in memory
    action_simulator_->saveKBSnapshot();

    // init open list (O), initially contains all nodes in partial order plan
    open_list_.clear();
    for(auto nit=original_plan_.nodes.begin(); nit!=original_plan_.nodes.end(); nit++) // nit=node iterator
        open_list_.push_back(nit->node_id);

    // init set of constraints (C)
    initConstraints(set_of_constraints_);

    // init set of ordered nodes (F)
    ordered_nodes_.clear();

    // init set of totally ordered plans (R)
    ordered_plans_.esterel_plans.clear();

    // if true, it means at least one valid execution alternative was found
    return orderNodes();
}

bool CSPExecGenerator::srvCB(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    ROS_INFO("Generating execution alternatives service is computing now");

    // boolean not needed, we could use it in future to make algorithm to produce
    // req.data

    if(!is_esterel_plan_received_) {
        ROS_ERROR("requires esterel plan input has not being received yet, can't generate exec alternatives");
        // set result of srv as failure
        res.success = false;
        return true;
    }

    if(generateFullyConnectedPlan())
    {
        // indicates that at least one valid execution was found
        res.success = true; // set result of srv as success
        ROS_INFO("Found valid execution(s)");

        // publish esterel graph array (multiple ways of executing plan)
        pub_set_of_solutions_.publish(ordered_plans_);
    }
    else
    {
        // indicates that no valid execution was found, means replanning is needed
        res.success = false; // set result of srv as failure
        ROS_INFO("No valid execution was found, replanning is needed");
    }

    // we don't need this msg but is part of the std srv..
    res.message = "empty msg";

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
    ROS_INFO("Node is going to initialize...");

    // create object of the node class (CSPExecGenerator)
    CSPExecGenerator csp_exec_generator_node;

    // setup node frequency
    double node_frequency = 1.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 1.0);
    ROS_INFO("Node will run at : %lf [hz]", node_frequency);
    ros::Rate loop_rate(node_frequency);

    ROS_INFO("Node initialized.");

    while (ros::ok())
    {
        // main loop function
        csp_exec_generator_node.update();

        // sleep to control the node frequency
        loop_rate.sleep();
    }

    return 0;
}
