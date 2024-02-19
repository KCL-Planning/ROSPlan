#include "rosplan_planning_system/PlannerInterface/TFDPlannerInterface.h"

namespace KCL_rosplan {

    /*-------------*/
    /* constructor */
    /*-------------*/

    TFDPlannerInterface::TFDPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
    }

    TFDPlannerInterface::~TFDPlannerInterface()
    {
        delete plan_server;
    }

    /*------------------*/
    /* Plan and process */
    /*------------------*/

    /**
     * passes the problem to the Planner; the plan to post-processing.
     */
    bool TFDPlannerInterface::runPlanner() {
        // path is based on the default installation of the Temporal Fast Downward
        std::string plannerPlanPath = data_path + "../../rosplan_planning_system/common/bin/tfd-src-0.4/downward/";
        std::string tfdOutputName = "tfdplan";

        // save problem to file for TFD
        saveProblem();

        // prepare the planner command line
        std::string commandString = prepareCommand();

        // delete old plans before running the planner
        ROS_INFO("KCL: (%s) Removing old TFD plans with (%s) name.", ros::this_node::getName().c_str(), tfdOutputName.c_str());
        std::string removeOldPlans = "cd " + plannerPlanPath + " && rm " + tfdOutputName + ".*";
        runCommand(removeOldPlans);

        // call the planer
        callPlanner(commandString);

        // get the most optimal plan in case there are many (tfdplan.[the highest number])
        std::string bestPlanCommand = "cd " + plannerPlanPath + " && ls | grep " + tfdOutputName + " | tail -1";
        std::string bestPlan = runCommand(bestPlanCommand.c_str());
        // get rid of carriage return
        if (bestPlan[bestPlan.size() - 1] == '\n') {
            bestPlan.erase(bestPlan.size() - 1);
        }

        // prepare command to copy plan
        std::string updatePlanCommand = "cp " + plannerPlanPath + bestPlan + " " + data_path + "plan.pddl";
        runCommand(updatePlanCommand.c_str());

        // check the planner solved the problem
        std::ifstream planfile;
        planfile.open((data_path + "plan.pddl").c_str());
        std::string line;
        std::stringstream ss;

        int curr, next;
        bool solved = false;
        double planDuration;

        while (std::getline(planfile, line)) {

            if (line.find("0.0", 0) != std::string::npos){
                solved = true;
            }
            planDuration = 0;
            ss.str("");
            do {
                if (line.length()<2)
                    break;
                ss << line << std::endl;
            } while (std::getline(planfile, line));
            
            planner_output = ss.str();

        }
        planfile.close();

        solvedMessages(solved);

        return solved;
    }

} // close namespace
