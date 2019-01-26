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

    // publications
    // array of esterel plans with different execution alternatives
    pub_set_of_solutions_ = nh_.advertise<rosplan_dispatch_msgs::EsterelPlanArray>("~set_of_solutions", 1);
    // partially ordered plan = original plan - conditional edges = plan with only interference edges
    pub_pop_ = nh_.advertise<rosplan_dispatch_msgs::EsterelPlan>("~partially_ordered_plan", 1);

    // services: compute different execution alternatives from a partially ordered esterel plan (a plan
    // with no conditional edges, but only interference edges)
    ros::ServiceServer service = nh_.advertiseService("gen_exec_alternatives", &CSPExecGenerator::srvCB, this);
}

CSPExecGenerator::~CSPExecGenerator()
{
    // shut down publishers and subscribers
    sub_esterel_plan_.shutdown();
    pub_set_of_solutions_.shutdown();
    pub_pop_.shutdown();
}

void CSPExecGenerator::esterelPlanCB(const rosplan_dispatch_msgs::EsterelPlan::ConstPtr& msg)
{
    original_fully_ordered_plan_ = *msg;
    is_esterel_plan_received_ = true;
}

rosplan_dispatch_msgs::EsterelPlan CSPExecGenerator::removeConditionalEdges(rosplan_dispatch_msgs::EsterelPlan plan)
{
    // remove conditional edges from a fully connected esterel plan received in original_fully_ordered_plan_

    // iterate over edges
    for(auto it=original_fully_ordered_plan_.edges.begin(); it!=original_fully_ordered_plan_.edges.end(); it++) {
        // discriminate by conditional edges
        if(it->edge_type == rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE)
            // delete elements which match criteria
            original_fully_ordered_plan_.edges.erase(it);
    }

    // publish the partially ordered plan, previous plan without conditional edges
    pub_pop_.publish(original_fully_ordered_plan_);
}

bool CSPExecGenerator::computeExecAlternatives()
{
    // remove conditional edges from plan
    partial_order_plan_ = removeConditionalEdges(original_fully_ordered_plan_);

    rosplan_dispatch_msgs::EsterelPlan valid_execution_alternative;

    rosplan_dispatch_msgs::EsterelPlanNode node;
    node.node_type = rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START;
    // node.node_type = rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END;
    // node.node_type = rosplan_dispatch_msgs::EsterelPlanNode::PLAN_START;
    node.node_id = 0; // integer
    node.name = ""; // str
        rosplan_dispatch_msgs::ActionDispatch action;
        action.action_id = 0; // int
        action.name = ""; // str
            diagnostic_msgs::KeyValue param;
            param.key = ""; // str
            param.value = ""; // str
        action.parameters.push_back(param);
        action.duration = 0.0; //float
        action.dispatch_time = 0.0; // float
    node.action = action;
    node.edges_out.push_back(0); // int array
    node.edges_in.push_back(0); // int array

    rosplan_dispatch_msgs::EsterelPlanEdge edge;
    edge.edge_type = rosplan_dispatch_msgs::EsterelPlanEdge::CONDITION_EDGE;
    // edge.edge_type = rosplan_dispatch_msgs::EsterelPlanEdge::START_END_ACTION_EDGE;
    // edge.edge_type = rosplan_dispatch_msgs::EsterelPlanEdge::INTERFERENCE_EDGE;
    edge.edge_id = 0; // int
    edge.edge_name = "";
    edge.signal_type = 0; // int
    edge.source_ids.push_back(0); // int array
    edge.sink_ids.push_back(0); // int
    edge.duration_lower_bound = 0.0; // float
    edge.duration_upper_bound = 0.0; // float

    valid_execution_alternative.nodes.push_back(node);
    valid_execution_alternative.edges.push_back(edge);

    // add plan to the set
    solution_set_.esterel_plans.push_back(valid_execution_alternative);

    // if true, it means at least one valid execution alternative was found
    return true;
}

bool CSPExecGenerator::srvCB(std_srvs::SetBool::Response &req, std_srvs::SetBool::Response &res)
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

    if(computeExecAlternatives())
    {
        // indicates that at least one valid execution was found
        res.success = true; // set result of srv as success
        ROS_INFO("Found valid execution(s)");

        // publish esterel graph array (multiple ways of executing plan)
        pub_set_of_solutions_.publish(solution_set_);
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

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "csp_exec_generator_node");
    ROS_INFO("Node is going to initialize...");

    // create object of the node class (CSPExecGenerator)
    CSPExecGenerator csp_exec_generator_node;
    ROS_INFO("Node initialized.");

    // wait for callbacks (incoming topics or service calls
    ros::spin();

    return 0;
}
