#include <ros/ros.h>
#include <vector>
#include <random>

#include "rosplan_action_interface/RPActionInterface.h"

#ifndef KCL_simulatedaction
#define KCL_simulatedaction

/**
 * This file defines the RPSimulatedActionInterface class.
 * RPSimulatedActionInterface is used to simulate synthetic actions (non physics based simulator)
 * 
 */
namespace KCL_rosplan {

	class RPSimulatedActionInterface: public RPActionInterface
	{

	private:

		double action_duration;
		double action_duration_stddev;
		double action_probability;

	public:

		/* constructor */
		RPSimulatedActionInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
