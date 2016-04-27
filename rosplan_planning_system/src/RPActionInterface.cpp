#include "rosplan_action_interface/RPActionInterface.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* run action interface */
	void RPActionInterface::runActionInterface() {

		ros::NodeHandle nh("~");

		// set action name
		nh.getParam("pddl_action_name", params.name);

		// fetch action params
		ros::ServiceClient client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/kcl_rosplan/get_domain_operator_details");
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = params.name;
		if(client.call(srv)) {
			params = srv.response.op.formula;
			op = srv.response.op;
		} else {
			ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for operator details, %s", params.name.c_str());
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
		std::map<std::string, std::string> boundParameters;
		for(size_t j=0; j<params.typed_parameters.size(); j++) {
			for(size_t i=0; i<msg->parameters.size(); i++) {
				if(params.typed_parameters[j].key == msg->parameters[i].key) {
					boundParameters[msg->parameters[i].key] = msg->parameters[i].value;
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

			// update knowledge base
			rosplan_knowledge_msgs::KnowledgeUpdateService updatePredSrv;
			updatePredSrv.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			
			// simple add effects
			for(int i=0; i<op.at_start_add_effects.size(); i++) {
				updatePredSrv.request.knowledge.attribute_name = op.at_start_add_effects[i].name;
				updatePredSrv.request.knowledge.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_start_add_effects[i].typed_parameters.size(); j++) {
					pair.key = op.at_start_add_effects[i].typed_parameters[j].key;
					pair.value = boundParameters[op.at_start_add_effects[i].typed_parameters[j].key];
					updatePredSrv.request.knowledge.values.push_back(pair);
				}
				update_knowledge_client.call(updatePredSrv);
			}

			for(int i=0; i<op.at_end_add_effects.size(); i++) {
				updatePredSrv.request.knowledge.attribute_name = op.at_end_add_effects[i].name;
				updatePredSrv.request.knowledge.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_end_add_effects[i].typed_parameters.size(); j++) {
					pair.key = op.at_end_add_effects[i].typed_parameters[j].key;
					pair.value = boundParameters[op.at_end_add_effects[i].typed_parameters[j].key];
					updatePredSrv.request.knowledge.values.push_back(pair);
				}
				update_knowledge_client.call(updatePredSrv);
			}

			// simple del effects
			updatePredSrv.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
			for(int i=0; i<op.at_start_del_effects.size(); i++) {
				updatePredSrv.request.knowledge.attribute_name = op.at_start_del_effects[i].name;
				updatePredSrv.request.knowledge.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_start_del_effects[i].typed_parameters.size(); j++) {
					pair.key = op.at_start_del_effects[i].typed_parameters[j].key;
					pair.value = boundParameters[op.at_start_del_effects[i].typed_parameters[j].key];
					updatePredSrv.request.knowledge.values.push_back(pair);
				}
				update_knowledge_client.call(updatePredSrv);
			}

			for(int i=0; i<op.at_end_del_effects.size(); i++) {
				updatePredSrv.request.knowledge.attribute_name = op.at_end_del_effects[i].name;
				updatePredSrv.request.knowledge.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_end_del_effects[i].typed_parameters.size(); j++) {
					pair.key = op.at_end_del_effects[i].typed_parameters[j].key;
					pair.value = boundParameters[op.at_end_del_effects[i].typed_parameters[j].key];
					updatePredSrv.request.knowledge.values.push_back(pair);
				}
				update_knowledge_client.call(updatePredSrv);
			}

			// sleep a little
			ros::Rate big_rate(0.5);
			big_rate.sleep();

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
