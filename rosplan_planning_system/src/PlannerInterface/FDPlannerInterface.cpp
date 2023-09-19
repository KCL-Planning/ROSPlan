#include "rosplan_planning_system/PlannerInterface/FDPlannerInterface.h"
#include <iostream>

namespace KCL_rosplan {

    /*-------------*/
    /* constructor */
    /*-------------*/

    FDPlannerInterface::FDPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
    }

    FDPlannerInterface::~FDPlannerInterface()
    {
        delete plan_server;
    }

    /*------------------*/
    /* Plan and process */
    /*------------------*/

    /**
     * passes the problem to the Planner; the plan to post-processing.
     */
    bool FDPlannerInterface::runPlanner() {

        // save problem to file for FD
        saveProblem();

        // prepare the planner command line
        std::string commandString = prepareCommand();

        // path is based on the default installation of the Fast Downward planner
        std::string updatePlan = "cp "+data_path+"../../rosplan_planning_system/common/bin/downward/fdplan.1"+" "+data_path+"plan.pddl";

        // call the planer
        callPlanner(commandString);

        // move plan to correct path
        runCommand(updatePlan.c_str());

        // check the planner solved the problem
        std::ifstream planfile;
        planfile.open((data_path + "plan.pddl").c_str());
        std::string line;
        std::stringstream ss;

        int curr, next;
        bool solved = false;
        double planDuration;

        while (std::getline(planfile, line)) {

            if (line.find("(", 0) != std::string::npos){
                solved = true;
            }
            planDuration = 0;
            ss.str("");
            while (std::getline(planfile, line)) {
                if (line.length()<2)break;
                if (line.find(";", 0) != std::string::npos){
                    break;
                }
                ss << line << " [0.001]" << std::endl;
            }
            planner_output = ss.str();

        }
        planfile.close();

        solvedMessages(solved);

        return solved;
    }

} // close namespace
