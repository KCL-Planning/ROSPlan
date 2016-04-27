#include "rosplan_action_interface/RPActionInterface.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* run action interface */
	void RPActionInterface::runActionInterface() {

		ros::NodeHandle nh("~");

		// set action name
		nh.getParam("pddl_action_name", params.name);

		// fetch action params
		ros::ServiceClient client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>("/kcl_rosplan/get_domain_operators");
		rosplan_knowledge_msgs::GetDomainOperatorService srv;
		if(client.call(srv)) {
			bool foundAction = false;
			for(int i=0; i<srv.response.operators.size(); i++) {
				if(params.name == srv.response.operators[i].name) {
					params = srv.response.operators[i];
					foundAction = true;
				}
			}
			if(!foundAction)
				ROS_ERROR("KCL: (RPActionInterface) operator does not exist in domain %s", params.name.c_str());
		} else {
			ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for operator parameters, %s", params.name.c_str());
		}

		// create PDDL info publisher
		pddl_action_parameters_pub = nh.advertise<rosplan_knowledge_msgs::DomainFormula>("/kcl_rosplan/pddl_action_parameters", 10, true);

		// create the action feedback publisher
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");

		// loop
		ros::Rate loopRate(1);
		ROS_INFO("KCL: (%s) Ready to receive", params.name.c_str());

		while(ros::ok()) {

			pddl_action_parameters_pub.publish(params);

			loopRate.sleep();
			ros::spinOnce();
		}
	}

	/* run action interface */
	void RPActionInterface::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// check action name
		if(0!=msg->name.compare(params.name)) return;
		ROS_INFO("KCL: (%s) action recieved", params.name.c_str());

		// check PDDL parameters
		std::vector<bool> found(params.typed_parameters.size(), false);
		for(size_t j=0; j<params.typed_parameters.size(); j++) {
			for(size_t i=0; i<msg->parameters.size(); i++) {
				if(params.typed_parameters[j].key == msg->parameters[i].key) {
					found[i] = true;
					break;
				}
			}
			if(!found[j]) {
				ROS_INFO("KCL: (%s) aborting action dispatch; malformed parameters, missing %s", params.name.c_str(), params.typed_parameters[j].key.c_str());
				return;
			}
		}

		// send feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		// publish feedback (enabled)
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);

		// call concrete implementation
		action_success = concreteCallback(msg);

		if(action_success) {

			// publish feedback (achieved)
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);

		} else {

			// publish feedback (failed)
			fb.status = "action failed";
			action_feedback_pub.publish(fb);
		}
	}

} // close namespace
