#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_LPG_planner_interface
#define KCL_LPG_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class LPGPlannerInterface: public PlannerInterface
	{
	private:

		/* runs external commands */
		std::string runCommand(std::string cmd);

	protected:

		bool runPlanner();

	public:

		LPGPlannerInterface(ros::NodeHandle& nh);
		virtual ~LPGPlannerInterface();
	};

} // close namespace

#endif
