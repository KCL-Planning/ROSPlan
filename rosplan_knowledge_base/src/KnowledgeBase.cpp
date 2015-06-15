#include "rosplan_knowledge_base/KnowledgeBase.h"

namespace KCL_rosplan {

	/*-----------------*/
	/* knowledge query */
	/*-----------------*/

	bool KnowledgeBase::queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res) {

		res.all_true = true;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
		for(iit = req.knowledge.begin(); iit!=req.knowledge.end(); iit++) {

			bool present = false;
			if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {

				// check if instance exists
				std::vector<std::string>::iterator sit;
				sit = find(domain_instances[iit->instance_type].begin(), domain_instances[iit->instance_type].end(), iit->instance_name);
				present = (sit!=domain_instances[iit->instance_type].end());
				
			} else if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_FUNCTION) {

				// check if function exists; TODO inequalities
				std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
				for(pit=domain_functions.begin(); pit!=domain_functions.end(); pit++) {
					if(KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
						present = true;
						pit = domain_functions.end();
					}
				}
			} else if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

				// check if fact is true
				std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
				for(pit=domain_attributes.begin(); pit!=domain_attributes.end(); pit++) {
					if(KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
						present = true;
						break;
					}
				}
			}

			if(!present) {
				res.all_true = false;
				res.false_knowledge.push_back(*iit);
			}
		}

		return true;
	}

	/*------------------*/
	/* knowledge update */
	/*------------------*/

	bool KnowledgeBase::updateKnowledge(rosplan_knowledge_msgs::KnowledgeUpdateService::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateService::Response &res) {

		if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE)
			addKnowledge(req.knowledge);
		else if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL)
			addMissionGoal(req.knowledge);
		else if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE)
			removeKnowledge(req.knowledge);
		else if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL)
			removeMissionGoal(req.knowledge);

		res.success = true;
		return true;
	}

	/*----------------*/
	/* removing items */
	/*----------------*/

	void KnowledgeBase::removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {		

			// search for instance
			std::vector<std::string>::iterator iit;
			for(iit = domain_instances[msg.instance_type].begin(); iit!=domain_instances[msg.instance_type].end(); iit++) {

				std::string name = *iit;

				if(name.compare(msg.instance_name)==0 || msg.instance_name.compare("")==0) {

					// remove instance from knowledge base
					ROS_INFO("KCL: (KB) Removing instance (%s, %s)", msg.instance_type.c_str(), (msg.instance_name.compare("")==0) ? "ALL" : msg.instance_name.c_str());
					iit = domain_instances[msg.instance_type].erase(iit);
					if(iit!=domain_instances[msg.instance_type].begin()) iit--;
					plan_filter.checkFilters(msg, false);

					// remove affected domain attributes
					std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
					for(pit=domain_attributes.begin(); pit!=domain_attributes.end(); pit++) {
						if(KnowledgeComparitor::containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing domain attribute (%s)", pit->attribute_name.c_str());
							plan_filter.checkFilters(*pit, false);
							pit = domain_attributes.erase(pit);
							if(pit!=domain_attributes.begin()) pit--;
							if(pit==domain_attributes.end()) break;
						}
					}

					// remove affected instance attributes
					for(pit=instance_attributes[name].begin(); pit!=instance_attributes[name].end(); pit++) {
						if(KnowledgeComparitor::containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing instance attribute (%s, %s)", name.c_str(), pit->attribute_name.c_str());
							plan_filter.checkFilters(*pit, false);
							pit = instance_attributes[name].erase(pit);
							if(pit!=instance_attributes[name].begin()) pit--;
							if(pit==instance_attributes[name].end()) break;
						}
					}

					// finish
					if(iit==domain_instances[msg.instance_type].end()) break;
				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_FUNCTION) {

			// remove domain attribute (function) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domain_functions.begin(); pit!=domain_functions.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					plan_filter.checkFilters(msg, false);
					pit = domain_functions.erase(pit);
					if(pit!=domain_functions.begin()) pit--;
					if(pit==domain_functions.end()) break;
				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

			// remove domain attribute (predicate) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domain_attributes.begin(); pit!=domain_attributes.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					plan_filter.checkFilters(msg, false);
					pit = domain_attributes.erase(pit);
					if(pit!=domain_attributes.begin()) pit--;
					if(pit==domain_attributes.end()) break;
				}
			}

		}
	}

	/**
	 * remove mission goal
	 */
	void KnowledgeBase::removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		bool changed = false;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator git;
		for(git=domain_goals.begin(); git!=domain_goals.end(); git++) {
			if(KnowledgeComparitor::containsKnowledge(msg, *git)) {
				ROS_INFO("KCL: (KB) Removing goal (%s)", msg.attribute_name.c_str());
				git = domain_goals.erase(git);
				if(git!=domain_goals.begin()) git--;
				if(git==domain_goals.end()) break;
			}
		}

		if(changed) {			
			rosplan_knowledge_msgs::Notification notMsg;
			notMsg.function = rosplan_knowledge_msgs::Notification::REMOVED;
			notMsg.knowledge_item = msg;
			plan_filter.notification_publisher.publish(notMsg);
		}
	}

	/*--------------*/
	/* adding items */
	/*--------------*/

	/*
	 * add an instance, domain predicate, or function to the knowledge base
	 */
	void KnowledgeBase::addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg) {
		
		if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {

			// check if instance is already in knowledge base
			std::vector<std::string>::iterator iit;
			iit = find(domain_instances[msg.instance_type].begin(), domain_instances[msg.instance_type].end(), msg.instance_name);

			// add instance
			if(iit==domain_instances[msg.instance_type].end()) {
				ROS_INFO("KCL: (KB) Adding instance (%s, %s)", msg.instance_type.c_str(), msg.instance_name.c_str());
				domain_instances[msg.instance_type].push_back(msg.instance_name);
				plan_filter.checkFilters(msg, true);
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

			// add domain attribute
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domain_attributes.begin(); pit!=domain_attributes.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit))
					return;
			}
			ROS_INFO("KCL: (KB) Adding domain attribute (%s)", msg.attribute_name.c_str());
			domain_attributes.push_back(msg);
			plan_filter.checkFilters(msg, true);

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_FUNCTION) {

			// add domain function
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domain_functions.begin(); pit!=domain_functions.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					// already added TODO value check
					ROS_INFO("KCL: (KB) Updating domain function (%s)", msg.attribute_name.c_str());
					pit->function_value = msg.function_value;
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding domain function (%s)", msg.attribute_name.c_str());
			domain_functions.push_back(msg);
			plan_filter.checkFilters(msg, true);
		}
	}

	/*
	 * add mission goal to knowledge base
	 */
	void KnowledgeBase::addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
		for(pit=domain_goals.begin(); pit!=domain_goals.end(); pit++) {
			if(KnowledgeComparitor::containsKnowledge(msg, *pit))
				return;
		}
		ROS_INFO("KCL: (KB) Adding mission goal (%s)", msg.attribute_name.c_str());
		domain_goals.push_back(msg);
	}

	/*----------------*/
	/* fetching items */
	/*----------------*/

	bool KnowledgeBase::getInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending %s getInstances", req.type_name.c_str());
	
		// fetch the instances of the correct type
		if(""==req.type_name) {
			std::map<std::string,std::vector<std::string> >::iterator iit;
			for(iit=domain_instances.begin(); iit != domain_instances.end(); iit++) {
				for(size_t j=0; j<iit->second.size(); j++)
					res.instances.push_back(iit->second[j]);
			}
		} else {
			std::map<std::string,std::vector<std::string> >::iterator iit;
			iit = domain_instances.find(req.type_name);
			if(iit != domain_instances.end()) {
				for(size_t j=0; j<iit->second.size(); j++)
					res.instances.push_back(iit->second[j]);
			}
		}

		return true;
	}

	bool KnowledgeBase::getDomainAttr(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getDomainAttr response for %s", req.predicate_name.c_str());

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<domain_attributes.size(); i++) {
			if(0==req.predicate_name.compare(domain_attributes[i].attribute_name) || ""==req.predicate_name)
				res.attributes.push_back(domain_attributes[i]);
		}

		// ...or fetch the knowledgeItems of the correct function
		for(size_t i=0; i<domain_functions.size(); i++) {
			if(0==req.predicate_name.compare(domain_functions[i].attribute_name) || ""==req.predicate_name)
				res.attributes.push_back(domain_functions[i]);
		}

		return true;
	}

	bool KnowledgeBase::getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getCurrentGoals response");

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<domain_goals.size(); i++) {
			res.attributes.push_back(domain_goals[i]);
		}

		return true;
	}

} // close namespace

/*-------------*/
/* main method */
/*-------------*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "KCL_knowledge_base");
	ros::NodeHandle n;

	KCL_rosplan::KnowledgeBase kb;

	// TESTING
	kb.domain_instances["robot"].push_back("kenny");
	// END TESTING */

	// query knowledge
	ros::ServiceServer queryServer = n.advertiseService("/kcl_rosplan/query_knowledge_base", &KCL_rosplan::KnowledgeBase::queryKnowledge, &kb);

	// update knowledge
	ros::ServiceServer updateServer = n.advertiseService("/kcl_rosplan/update_knowledge_base", &KCL_rosplan::KnowledgeBase::updateKnowledge, &kb);

	// environment services
	ros::ServiceServer instanceServer = n.advertiseService("/kcl_rosplan/get_instances", &KCL_rosplan::KnowledgeBase::getInstances, &kb);
	ros::ServiceServer domainServer = n.advertiseService("/kcl_rosplan/get_domain_attributes", &KCL_rosplan::KnowledgeBase::getDomainAttr, &kb);
	ros::ServiceServer goalServer = n.advertiseService("/kcl_rosplan/get_current_goals", &KCL_rosplan::KnowledgeBase::getCurrentGoals, &kb);

	// filter
	kb.plan_filter.notification_publisher = n.advertise<rosplan_knowledge_msgs::Notification>("/kcl_rosplan/notification", 10, true);
	ros::Subscriber planningFilterSub = n.subscribe("/kcl_rosplan/plan_filter", 100, &KCL_rosplan::PlanFilter::planningFilterCallback, &kb.plan_filter);
	ros::Subscriber missionFilterSub = n.subscribe("/kcl_rosplan/plan_filter", 100, &KCL_rosplan::PlanFilter::missionFilterCallback, &kb.plan_filter);

	// wait for and clear mongoDB 
	ROS_INFO("KCL: (KB) Waiting for MongoDB");
	ros::service::waitForService("/message_store/delete",-1);
	system("mongo message_store --eval \"printjson(db.message_store.remove())\"");

	ROS_INFO("KCL: (KB) Ready to receive");
	ros::spin();

	return 0;
}
