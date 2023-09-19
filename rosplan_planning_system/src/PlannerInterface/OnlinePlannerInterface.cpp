//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#include <rosplan_planning_system/PlannerInterface/OnlinePlannerInterface.h>

namespace KCL_rosplan {

    OnlinePlannerInterface::OnlinePlannerInterface(ros::NodeHandle &nh): PlannerInterface(nh) {
        // start planning action server
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
            ROS_INFO("ROSPlan: (%s) A plan is already being computed.", ros::this_node::getName().c_str());
            return false;
        }
        else if (planner.joinable()) planner.join();

        // save problem to file for planner
        saveProblem();

        // prepare the planner command line
        std::string commandString = prepareCommand();
        // std::string commandString = str + " > " + data_path + "plan.pddl";

        // call the planer
        ROS_INFO("ROSPlan: (%s) (%s) Running: %s in a new thread", ros::this_node::getName().c_str(), problem_name.c_str(),
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
