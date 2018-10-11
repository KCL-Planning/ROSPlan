//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#ifndef ROSPLAN_PLANNING_SYSTEM_ONLINEPLANNERINTERFACE_H
#define ROSPLAN_PLANNING_SYSTEM_ONLINEPLANNERINTERFACE_H

#include "PlannerInterface.h"
#include <fstream>
#include <thread>

namespace KCL_rosplan {

    class OnlinePlannerInterface : public PlannerInterface {
    private:

        /* runs external commands */
        std::string runCommand(std::string cmd);
        std::thread planner;
        bool planner_running;

    protected:

        bool runPlanner();

    public:

        OnlinePlannerInterface(ros::NodeHandle& nh);
        ~OnlinePlannerInterface();
    };
}

#endif //ROSPLAN_PLANNING_SYSTEM_ONLINEPLANNERINTERFACE_H
