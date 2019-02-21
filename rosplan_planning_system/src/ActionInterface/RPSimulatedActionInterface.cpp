#include "rosplan_action_interface/RPSimulatedActionInterface.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* constructor */
	RPSimulatedActionInterface::RPSimulatedActionInterface(ros::NodeHandle &nh) {
		action_duration = 0.0;
		action_probability = 1.0;
		nh.getParam("action_duration", action_duration);
		nh.getParam("action_duration_stddev", action_duration_stddev);
		nh.getParam("action_probability", action_probability);
	}

	/* action dispatch callback */
	bool RPSimulatedActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// wait for some time
		if(action_duration_stddev > 0) {
            std::default_random_engine generator(ros::Time::now().toSec());
            std::normal_distribution<double> distribution(action_duration, action_duration_stddev);
            double d = distribution(generator);
            if(d < 0) d = 0;
    		ROS_INFO("KCL: (%s) Action completing with probability %f and duration %f", params.name.c_str(), action_probability, d);
		} else if(action_duration > 0) {
    		ROS_INFO("KCL: (%s) Action completing with probability %f and duration %f", params.name.c_str(), action_probability, action_duration);
			ros::Rate wait = 1.0 / action_duration;
			wait.sleep();
        } else {
    		ROS_INFO("KCL: (%s) Action completing with probability %f and duration %f", params.name.c_str(), action_probability, action_duration);
        }

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
