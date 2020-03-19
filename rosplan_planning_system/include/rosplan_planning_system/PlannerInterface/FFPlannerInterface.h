#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_FF_planner_interface
#define KCL_FF_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class FFPlannerInterface: public PlannerInterface
	{
	private:

		bool use_ffha;

		/* runs external commands */
		std::string runCommand(std::string cmd);

	protected:

		bool runPlanner();

	public:

		FFPlannerInterface(ros::NodeHandle& nh);
		virtual ~FFPlannerInterface();
	};

} // close namespace

#endif
