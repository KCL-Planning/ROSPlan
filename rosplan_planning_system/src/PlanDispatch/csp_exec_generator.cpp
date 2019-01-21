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

#include <rosplan_planning_system/PlanDispatch/csp_exec_generator.h>
#include <string>
#include <vector>

CSPExecGenerator::CSPExecGenerator() : nh_("~"), is_esterel_plan_received_(false)
{
    // subscriptions
    sub_esterel_plan_ = nh_.subscribe("esterel_plan", 1, &CSPExecGenerator::esterelPlanCB, this);

    // publications
    pub_set_of_solutions_ = nh_.advertise<rosplan_dispatch_msgs::EsterelPlanArray>("~set_of_solutions", 1);

    // services
    ros::ServiceServer service = nh_.advertiseService("gen_exec_alternatives", &CSPExecGenerator::srvCB, this);
    
    // querying parameters from parameter server
    getParams();
}

CSPExecGenerator::~CSPExecGenerator()
{
    // shut down publishers and subscribers
    sub_esterel_plan_.shutdown();
    pub_set_of_solutions_.shutdown();
}

void CSPExecGenerator::getParams()
{
//     // setup script default arguments
//     std::vector<std::string> default_args;
//     default_args.push_back("no_args");
// 
//     nh_.param<std::string>("script_path", full_path_to_script_, "/home/user/my_script.sh");
//     nh_.param<std::vector<std::string> >("script_arguments", script_arguments_, default_args);
// 
//     // informing the user about the parameters which will be used
//     ROS_INFO("Script path : %s", full_path_to_script_.c_str());
// 
//     ROS_INFO("Script will run with the following arguments :");
//     for (int i = 0; i < script_arguments_.size() ; i++)
//     {
//         ROS_INFO("arg %d : %s", i + 1, script_arguments_.at(i).c_str());
//     }
}

void CSPExecGenerator::esterelPlanCB(const rosplan_dispatch_msgs::EsterelPlan::ConstPtr& msg)
{
    esterel_plan_msg_ = *msg;
    is_esterel_plan_received_ = true;
}

bool CSPExecGenerator::compute_exec_alternatives()
{
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
    
    // TODO: implement

    if(compute_exec_alternatives())
    {
        // indicates that at least one valid execution was found
        res.success = true;
        ROS_INFO("Found valid execution(s)");

        // publish esterel graph array (multiple ways of executing plan)
        pub_set_of_solutions_.publish(solution_set_);
    }
    else
    {
        // indicates that no valid execution was found, means replanning is needed
        res.success = false;
        ROS_INFO("No valid execution was found, replanning is needed");
    }

    // we don't need this msg but is part of the std srv..
    res.message = "no msg";

    ROS_INFO("Generating execution alternatives service has finished");

    return true;
}

void CSPExecGenerator::update()
{
    // listen to callbacks
    ros::spinOnce();

    if (!is_esterel_plan_received_) return;

    // reset flag
    is_esterel_plan_received_ = false;

    // TODO: perform algorithm
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "csp_exec_generator_node");

    ROS_INFO("Node is going to initialize...");

    // create object of the node class (CSPExecGenerator)
    CSPExecGenerator csp_exec_generator_node;

    // setup node frequency
    double node_frequency = 10.0;
    ros::NodeHandle nh("~");
    nh.param("node_frequency", node_frequency, 10.0);
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
