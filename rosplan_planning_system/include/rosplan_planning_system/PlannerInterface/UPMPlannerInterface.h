#include "PlannerInterface.h"
#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_UPM_planner_interface
#define KCL_UPM_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class UPMPlannerInterface: public PlannerInterface
	{
	private:

		/* runs external commands */
		std::string runCommand(std::string cmd);

	protected:

        std::string input_problem_path;
		bool runPlanner();

	public:

		UPMPlannerInterface(ros::NodeHandle& nh);
		virtual ~UPMPlannerInterface();
	};

} // close namespace

#endif
