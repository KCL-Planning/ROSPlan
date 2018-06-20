#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_POPF_planner_interface
#define KCL_POPF_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class POPFPlannerInterface: public PlannerInterface
	{
	private:

		/* runs external commands */
		std::string runCommand(std::string cmd);

	protected:

		bool runPlanner();

	public:

		POPFPlannerInterface(ros::NodeHandle& nh);
		virtual ~POPFPlannerInterface();
	};

} // close namespace

#endif
