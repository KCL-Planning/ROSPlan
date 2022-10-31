#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_PPRP_planner_interface
#define KCL_PPRP_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class PPRPPlannerInterface: public PlannerInterface
	{
	private:

		/* runs external commands */
		std::string runCommand(std::string cmd);

	protected:

		bool runPlanner();

	public:

        PPRPPlannerInterface(ros::NodeHandle& nh);
		virtual ~PPRPPlannerInterface();
	};

} // close namespace

#endif
