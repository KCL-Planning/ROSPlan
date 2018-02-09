#include "rosplan_action_interface/RPSimulatedActionInterface.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPSimulatedActionInterface::RPSimulatedActionInterface(ros::NodeHandle &nh) {
		action_duration = action_probability = 1.0;
		nh.getParam("action_duration", action_duration);
		nh.getParam("action_probability", action_probability);
	}

	/* action dispatch callback */
	bool RPSimulatedActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		ROS_INFO("KCL: (%s) Action completing with probability %f", params.name.c_str(), action_probability);

		// wait for some time
		ros::Rate wait = (action_duration);
		wait.sleep();

		// complete the action
		return (rand() % 100) <= (100 * action_probability);
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_simulated_action", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPSimulatedActionInterface rpsa(nh);

		rpsa.runActionInterface();

		return 0;
	}
