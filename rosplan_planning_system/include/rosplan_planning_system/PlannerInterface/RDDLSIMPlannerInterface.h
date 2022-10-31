//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 28/09/18.
//

#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <string>
#include <streambuf>

#ifndef KCL_RDDLSIM_planner_interface
#define KCL_RDDLSIM_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan {

	class RDDLSIMPlannerInterface: public PlannerInterface
	{
	private:

		/* runs external commands */
		std::string runCommand(std::string cmd);

	protected:

		bool runPlanner();

	public:

        RDDLSIMPlannerInterface(ros::NodeHandle& nh);
		virtual ~RDDLSIMPlannerInterface();
	};

} // close namespace

#endif
