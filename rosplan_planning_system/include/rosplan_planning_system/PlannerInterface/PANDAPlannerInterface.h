#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_PANDA_planner_interface
#define KCL_PANDA_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

    class PANDAPlannerInterface: public PlannerInterface
    {
    private:

    protected:

        bool runPlanner();

    public:

        PANDAPlannerInterface(ros::NodeHandle& nh);
        virtual ~PANDAPlannerInterface();
    };

} // close namespace

#endif
