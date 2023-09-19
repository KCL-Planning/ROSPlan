#include "rosplan_planning_system/PlannerInterface/LPGPlannerInterface.h"

namespace KCL_rosplan {

    /*-------------*/
    /* constructor */
    /*-------------*/

    LPGPlannerInterface::LPGPlannerInterface(ros::NodeHandle& nh): PlannerInterface(nh) {
    }

    LPGPlannerInterface::~LPGPlannerInterface()
    {
        delete plan_server;
    }

    /*------------------*/
    /* Plan and process */
    /*------------------*/

    /**
     * passes the problem to the Planner; the plan to post-processing.
     */
    bool LPGPlannerInterface::runPlanner() {

        // save problem to file for LPG
        saveProblem();

        // prepare the planner command line
        std::string commandString = prepareCommand();
        std::size_t oit = commandString.find("-out ");

        //change output path of the planner to data_path/lpgplan
        ROS_INFO("KCL: (%s) (%s) Changing '-out' to 'data_path/lpgplan'. !!! '-out' must be the last parameter in the planner_command !!!", ros::this_node::getName().c_str(), problem_name.c_str());
        if(oit!=std::string::npos) commandString.replace(oit,500,"-out "+data_path+"lpgplan");

        //copy the .SOL output plan to data_path/plan.pddl
        std::string updatePlan = "cp "+data_path+"lpgplan.SOL"+" "+data_path+"plan.pddl";

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
            }else if (line.find("; ", 0) == std::string::npos) {
                // consume useless lines
            }
            planDuration = 0;

            // extract second block of uninterrupted lines (the actual plan) from .SOL file content
            if ((line.length()>1)){
                ss.str("");
            }
            while (std::getline(planfile, line)) {
                if (line.length()<2)
                    break;

                ss << line << std::endl;
            }
            planner_output = ss.str();

        }
        planfile.close();

        solvedMessages(solved);

        return solved;
    }

} // close namespace
