#include "rosplan_action_interface/RPActionInterface.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {

	/* run action interface */
	void RPActionInterface::runActionInterface() {

		ros::NodeHandle nh("~");

		// set action name
		nh.getParam("pddl_action_name", params.name);

		// knowledge base services
		std::string kb = "knowledge_base";
		nh.getParam("knowledge_base", kb);

		// fetch action params
		std::stringstream ss;
		ss << "/" << kb << "/domain/operator_details";
		ros::service::waitForService(ss.str(),ros::Duration(20));
		ros::ServiceClient client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str());
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = params.name;
		if(client.call(srv)) {
			params = srv.response.op.formula;
			op = srv.response.op;
		} else {
			ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for operator details, %s", params.name.c_str());
			return;
		}

		// collect predicates from operator description
		std::vector<std::string> predicateNames;

		// effects
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator pit = op.at_start_add_effects.begin();
		for(; pit!=op.at_start_add_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_start_del_effects.begin();
		for(; pit!=op.at_start_del_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_add_effects.begin();
		for(; pit!=op.at_end_add_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_del_effects.begin();
		for(; pit!=op.at_end_del_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		// simple conditions
		pit = op.at_start_simple_condition.begin();
		for(; pit!=op.at_start_simple_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.over_all_simple_condition.begin();
		for(; pit!=op.over_all_simple_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_simple_condition.begin();
		for(; pit!=op.at_end_simple_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		// negative conditions
		pit = op.at_start_neg_condition.begin();
		for(; pit!=op.at_start_neg_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.over_all_neg_condition.begin();
		for(; pit!=op.over_all_neg_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_neg_condition.begin();
		for(; pit!=op.at_end_neg_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		// fetch and store predicate details
		ss.str("");
		ss << "/" << kb << "/domain/predicate_details";
		ros::service::waitForService(ss.str(),ros::Duration(20));
		ros::ServiceClient predClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>(ss.str());
		std::vector<std::string>::iterator nit = predicateNames.begin();
		for(; nit!=predicateNames.end(); nit++) {
			if (predicates.find(*nit) != predicates.end()) continue;
			if (*nit == "=" || *nit == ">" || *nit == "<" || *nit == ">=" || *nit == "<=") continue;
			rosplan_knowledge_msgs::GetDomainPredicateDetailsService predSrv;
			predSrv.request.name = *nit;
			if(predClient.call(predSrv)) {
				predicates.insert(std::pair<std::string, rosplan_knowledge_msgs::DomainFormula>(*nit, predSrv.response.predicate));
			} else {
				ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for predicate details, %s", params.name.c_str());
				return;
			}
		}

		// create PDDL info publisher
		ss.str("");
		ss << "/" << kb << "/pddl_action_parameters";
		pddl_action_parameters_pub = nh.advertise<rosplan_knowledge_msgs::DomainFormula>(ss.str(), 10, true);

		// create the action feedback publisher
		std::string aft = "default_feedback_topic";
		nh.getParam("action_feedback_topic", aft);
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>(aft, 10, true);

		// knowledge interface
		ss.str("");
		ss << "/" << kb << "/update_array";
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());

		// listen for action dispatch
		std::string adt = "default_dispatch_topic";
		nh.getParam("action_dispatch_topic", adt);
		ros::Subscriber ds = nh.subscribe(adt, 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, this);

		// loop
		ros::Rate loopRate(50);
		int counter = 0;
		ROS_INFO("KCL: (%s) Ready to receive", params.name.c_str());

		while(ros::ok()) {

			counter++;
			if(counter==20) {
				pddl_action_parameters_pub.publish(params);
				counter = 0;
			}

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
					found[j] = true;
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

		{
			// update knowledge base
			rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;
			
			// simple START del effects
			for(int i=0; i<op.at_start_del_effects.size(); i++) {
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item.attribute_name = op.at_start_del_effects[i].name;
				item.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_start_del_effects[i].typed_parameters.size(); j++) {
					pair.key = predicates[op.at_start_del_effects[i].name].typed_parameters[j].key;
					pair.value = boundParameters[op.at_start_del_effects[i].typed_parameters[j].key];
					item.values.push_back(pair);
				}
				updatePredSrv.request.knowledge.push_back(item);
				updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
			}

			// simple START add effects
			for(int i=0; i<op.at_start_add_effects.size(); i++) {
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item.attribute_name = op.at_start_add_effects[i].name;
				item.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_start_add_effects[i].typed_parameters.size(); j++) {
					pair.key = predicates[op.at_start_add_effects[i].name].typed_parameters[j].key;
					pair.value = boundParameters[op.at_start_add_effects[i].typed_parameters[j].key];
					item.values.push_back(pair);
				}
				updatePredSrv.request.knowledge.push_back(item);
				updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
			}

			if(updatePredSrv.request.knowledge.size()>0 && !update_knowledge_client.call(updatePredSrv))
				ROS_INFO("KCL: (%s) failed to update PDDL model in knowledge base", params.name.c_str());
		}

		// call concrete implementation
		action_success = concreteCallback(msg);

		if(action_success) {

			// update knowledge base
			rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;

			// simple END del effects
			for(int i=0; i<op.at_end_del_effects.size(); i++) {
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item.attribute_name = op.at_end_del_effects[i].name;
				item.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_end_del_effects[i].typed_parameters.size(); j++) {
					pair.key = predicates[op.at_end_del_effects[i].name].typed_parameters[j].key;
					pair.value = boundParameters[op.at_end_del_effects[i].typed_parameters[j].key];
					item.values.push_back(pair);
				}
				updatePredSrv.request.knowledge.push_back(item);
				updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
			}

			// simple END add effects
			for(int i=0; i<op.at_end_add_effects.size(); i++) {
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item.attribute_name = op.at_end_add_effects[i].name;
				item.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_end_add_effects[i].typed_parameters.size(); j++) {
					pair.key = predicates[op.at_end_add_effects[i].name].typed_parameters[j].key;
					pair.value = boundParameters[op.at_end_add_effects[i].typed_parameters[j].key];
					item.values.push_back(pair);
				}
				updatePredSrv.request.knowledge.push_back(item);
				updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
			}

			if(updatePredSrv.request.knowledge.size()>0 && !update_knowledge_client.call(updatePredSrv))
				ROS_INFO("KCL: (%s) failed to update PDDL model in knowledge base", params.name.c_str());

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
