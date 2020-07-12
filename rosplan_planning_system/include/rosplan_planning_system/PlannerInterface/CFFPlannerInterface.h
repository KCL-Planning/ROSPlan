#include <string>
#include <fstream>
#include "PlannerInterface.h"

#ifndef KCL_FF_planner_interface
#define KCL_FF_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

    class CFFPlannerInterface: public PlannerInterface
    {
    private:

        void clearPreviousPlan();
        void saveProblemToFileIfNeeded();        
        void callExternalPlanner();
        std::string runCommand(std::string cmd);        
        bool isPlanSolved(std::ifstream &plan_file);
        void convertPlanToPopfFormat(std::ifstream &plan_file);
        void savePlanInPopfFormatToFile();
        bool parsePlan();

    protected:

        bool runPlanner();

    public:

        CFFPlannerInterface(ros::NodeHandle& nh);
        virtual ~CFFPlannerInterface();
    };

}

#endif
