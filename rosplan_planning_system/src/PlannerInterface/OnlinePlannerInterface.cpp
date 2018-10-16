//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#include <rosplan_planning_system/PlannerInterface/OnlinePlannerInterface.h>

#include "rosplan_planning_system/PlannerInterface/OnlinePlannerInterface.h"

namespace KCL_rosplan {

    OnlinePlannerInterface::OnlinePlannerInterface(ros::NodeHandle &nh) {
        node_handle = &nh;

        plan_server = new actionlib::SimpleActionServer<rosplan_dispatch_msgs::PlanAction>((*node_handle), "start_planning", boost::bind(&PlannerInterface::runPlanningServerAction, this, _1), false);

        // publishing raw planner output
        std::string plannerTopic = "planner_output";
        node_handle->getParam("planner_topic", plannerTopic);
        plan_publisher = node_handle->advertise<std_msgs::String>(plannerTopic, 1, true);

        // start planning action server
        plan_server->start();
        planner_running = false;
        get_planner_params = nh.advertiseService("get_planning_params", &KCL_rosplan::OnlinePlannerInterface::getPlannerParams, this);
    }

    OnlinePlannerInterface::~OnlinePlannerInterface() {
        delete plan_server;
    }

    /**
	 * Runs external commands
	 */
    std::string OnlinePlannerInterface::runCommand(std::string cmd) {
        planner_running = true;
        std::string data;
        FILE *stream;
        char buffer[1000];
        stream = popen(cmd.c_str(), "r");
        while ( fgets(buffer, 1000, stream) != NULL )
            data.append(buffer);
        pclose(stream);
        planner_running = false;
        return data;
    }

    bool OnlinePlannerInterface::runPlanner() {
        if (planner_running) {
            ROS_INFO("KCL: (%s) A plan is already being computed.", ros::this_node::getName().c_str());
            return false;
        }
        else if (planner.joinable()) planner.join();

        // save problem to file for planner
        if (use_problem_topic && problem_instance_received) {
            ROS_INFO("KCL: (%s) (%s) Writing problem to file.", ros::this_node::getName().c_str(),
                     problem_name.c_str());
            std::ofstream dest;
            dest.open((problem_path).c_str());
            dest << problem_instance;
            dest.close();
        }

        // prepare the planner command line
        std::string str = planner_command;
        std::size_t dit = str.find("DOMAIN");
        if (dit != std::string::npos) str.replace(dit, 6, domain_path);
        std::size_t pit = str.find("PROBLEM");
        if (pit != std::string::npos) str.replace(pit, 7, problem_path);
        std::string commandString = str + " > " + data_path + "plan.pddl";

        // call the planer
        ROS_INFO("KCL: (%s) (%s) Running: %s in a new thread", ros::this_node::getName().c_str(), problem_name.c_str(),
                 commandString.c_str());

        // Launch thread and return, as the dispatcher will take care of the planning
        planner = std::thread(&OnlinePlannerInterface::runCommand, this, commandString);
        return true;
    }

    bool OnlinePlannerInterface::getPlannerParams(rosplan_dispatch_msgs::GetPlanningParamsRequest &req,
                                                  rosplan_dispatch_msgs::GetPlanningParamsResponse &res) {
        res.domain_path = domain_path;
        res.problem_path = problem_path;
        res.planner_ready = planner_running;
        return true;
    }
}


/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    srand (static_cast <unsigned> (time(0)));

    ros::init(argc,argv,"rosplan_planner_interface");
    ros::NodeHandle nh("~");

    KCL_rosplan::OnlinePlannerInterface pi(nh);

    // subscribe to problem instance
    std::string problemTopic = "problem_instance";
    nh.getParam("problem_topic", problemTopic);
    ros::Subscriber problem_sub = nh.subscribe(problemTopic, 1, &KCL_rosplan::PlannerInterface::problemCallback, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

    // start the planning services
    ros::ServiceServer service1 = nh.advertiseService("planning_server", &KCL_rosplan::PlannerInterface::runPlanningServerDefault, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));
    ros::ServiceServer service2 = nh.advertiseService("planning_server_params", &KCL_rosplan::PlannerInterface::runPlanningServerParams, dynamic_cast<KCL_rosplan::PlannerInterface*>(&pi));

    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}