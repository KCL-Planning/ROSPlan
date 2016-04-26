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
		while(ros::ok()) {

			pddl_action_parameters_pub.publish(params);

			loopRate.sleep();
			ros::spinOnce();
		}
	}

} // close namespace
