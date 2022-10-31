#include "rosplan_action_interface/RPSimulatedActionInterface.h"

/* The implementation of RPSimulatedActionInterface.h */
namespace KCL_rosplan {

	/* constructor */
	RPSimulatedActionInterface::RPSimulatedActionInterface(ros::NodeHandle &nh) {
		action_duration = -1.0;        // use duration calculated on plan by default
		action_duration_stddev = 0.0;  // wait exactly action duration by default
		action_probability = 1.0;
		nh.getParam("action_duration", action_duration);
		nh.getParam("action_duration_stddev", action_duration_stddev);
		nh.getParam("action_probability", action_probability);
	}

	/* action dispatch callback */
	bool RPSimulatedActionInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// wait for some time, plan-calculated by default
        double duration = msg->duration;
        if(action_duration >= 0) {
            // use time provided for the simulated action
            duration = action_duration;
        }

        if(action_duration_stddev > 0) {
            std::default_random_engine generator(ros::WallTime::now().toSec());
            std::normal_distribution<double> distribution(duration, action_duration_stddev);
            duration = std::max(distribution(generator), 0.0);
        }

        ROS_INFO("KCL: (%s) Action completing with probability %g and duration %g", params.name.c_str(), action_probability, duration);
        if(duration > 0) {
            ros::Rate wait = 1.0 / duration;
            wait.sleep();
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
