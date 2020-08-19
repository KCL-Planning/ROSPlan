// Created by Sebastian Stock <sebastian.stock@dfki.de> on 2020-08-17

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_CHIMP_planner_interface
#define KCL_CHIMP_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class CHIMPPlannerInterface: public PlannerInterface
	{
	private:

		/* runs external commands */
		std::string runCommand(std::string cmd);

		/* create command line for the planner */
		std::string prepareCommand();
		/* write the problem instance to problem_path file */
		void writeProblemToFile();

	protected:

		bool runPlanner();

	public:

		CHIMPPlannerInterface(ros::NodeHandle& nh);
		virtual ~CHIMPPlannerInterface();
	};

} // close namespace

#endif
