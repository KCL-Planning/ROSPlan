#include "rosplan_planning_system/PlannerInterface/UPMPlannerInterface.h"

namespace KCL_rosplan {

    /*-------------*/
    /* constructor */
    /*-------------*/

    UPMPlannerInterface::UPMPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
    }

    UPMPlannerInterface::~UPMPlannerInterface()
    {
        delete plan_server;
    }

    /*------------------*/
    /* Plan and process */
    /*------------------*/

    /**
     * passes the problem to the Planner; the plan to post-processing.
     */
    bool UPMPlannerInterface::runPlanner() {

        // save problem to file for UPM
        saveProblem();

        // prepare the planner command line
        std::string commandString = prepareCommand();


        // create the plan path from domain_path (problem file must be named problem.pddl)
        std::stringstream sspp;
        for (unsigned i=0; i<domain_path.length() - 11; ++i)
        {
            sspp << domain_path.at(i);
        }
        sspp << "problem_plan.pddl";

        std::string updatePlan = "cp "+sspp.str()+" "+data_path+"plan.pddl";


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

            if (line.find("0.0", 0) != std::string::npos){
                solved = true;
            }
            ss.str("");
            if (line.find("; ", 0) == std::string::npos) {
                    ss << line << std::endl;
            }
            planner_output = ss.str();
        }
        planfile.close();

        solvedMessages(solved);

        return solved;
    }

} // close namespace
