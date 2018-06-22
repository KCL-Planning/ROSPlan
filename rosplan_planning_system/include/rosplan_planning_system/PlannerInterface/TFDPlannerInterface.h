#include "PlannerInterface.h"
#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_TFD_planner_interface
#define KCL_TFD_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class TFDPlannerInterface: public PlannerInterface
	{
	private:

		/* runs external commands */
		std::string runCommand(std::string cmd);

	protected:

		bool runPlanner();

	public:

		TFDPlannerInterface(ros::NodeHandle& nh);
		virtual ~TFDPlannerInterface();
	};

} // close namespace

#endif
