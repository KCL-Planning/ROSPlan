#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "rosplan_knowledge_base/KnowledgeBase.h"

namespace KCL_rosplan {


	/*-----------------*/
	/* knowledge query */
	/*-----------------*/

	bool KnowledgeBase::queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res) {

		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
		res.all_true = true;
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
					if(sameKnowledge(*iit, *pit)) {
						present = true;
						pit = domain_functions.end();
					}
				}

			} else if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

				// check if fact is true
				std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
				for(pit=domain_attributes.begin(); pit!=domain_attributes.end(); pit++) {
					if(sameKnowledge(*iit, *pit)) {
						present = true;
						pit = domain_attributes.end();
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
					checkFilters(msg, false);
					domain_instances[msg.instance_type].erase(iit);

					// remove affected domain attributes
					std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
					for(pit=domain_attributes.begin(); pit!=domain_attributes.end(); pit++) {
						if(containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing domain attribute (%s)", pit->attribute_name.c_str());
							checkFilters(*pit, false);
							pit = domain_attributes.erase(pit);
							if(pit==domain_attributes.end()) break;
						}
					}

					// remove affected instance attributes
					for(pit=instance_attributes[name].begin(); pit!=instance_attributes[name].end(); pit++) {
						if(containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing instance attribute (%s, %s)", name.c_str(), pit->attribute_name.c_str());
							checkFilters(*pit, false);
							pit = instance_attributes[name].erase(pit);
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
				if(sameKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					checkFilters(msg, false);
					pit = domain_functions.erase(pit);
					if(pit==domain_functions.end()) break;
				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

			// remove domain attribute (predicate) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domain_attributes.begin(); pit!=domain_attributes.end(); pit++) {
				if(sameKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					checkFilters(msg, false);
					pit = domain_attributes.erase(pit);
					if(pit==domain_attributes.end()) break;
				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE_ATTRIBUTE) {

			// remove instance attribute (non-symbolic) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=instance_attributes[msg.instance_name].begin(); pit!=instance_attributes[msg.instance_name].end(); pit++) {
				if(sameKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing instance attribute (%s, %s)", msg.instance_name.c_str(), msg.attribute_name.c_str());
					checkFilters(msg, false);
					pit = instance_attributes[msg.instance_name].erase(pit);
					if(pit==instance_attributes[msg.instance_name].end()) break;
				}
			}
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

			if(iit==domain_instances[msg.instance_type].end()) {
				// add instance
				ROS_INFO("KCL: (KB) Adding instance (%s, %s)", msg.instance_type.c_str(), msg.instance_name.c_str());
				domain_instances[msg.instance_type].push_back(msg.instance_name);
				checkFilters(msg, true);
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

			// add domain attribute
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domain_attributes.begin(); pit!=domain_attributes.end(); pit++) {
				if(sameKnowledge(msg, *pit)) {
					// already added
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding domain attribute (%s)", msg.attribute_name.c_str());
			domain_attributes.push_back(msg);
			checkFilters(msg, true);

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE_ATTRIBUTE) {

			// add instance attribute
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=instance_attributes[msg.instance_name].begin(); pit!=instance_attributes[msg.instance_name].end(); pit++) {
				if(sameKnowledge(msg, *pit)) {
					// already added
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding instance attribute (%s, %s)", msg.instance_name.c_str(), msg.attribute_name.c_str());
			instance_attributes[msg.instance_name].push_back(msg);
			checkFilters(msg, true);

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_FUNCTION) {

			// add domain function
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domain_functions.begin(); pit!=domain_functions.end(); pit++) {
				if(sameKnowledge(msg, *pit)) {
					// already added
					ROS_INFO("KCL: (KB) Updating domain function (%s)", msg.attribute_name.c_str());
					pit->function_value = msg.function_value;
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding domain function (%s)", msg.attribute_name.c_str());
			domain_functions.push_back(msg);
			checkFilters(msg, true);
		}
	}

	/*
	 * add mission goal to knowledge base
	 */
	void KnowledgeBase::addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {
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
		std::map<std::string,std::vector<std::string> >::iterator iit;
		iit = domain_instances.find(req.type_name);
		if(iit != domain_instances.end()) {
			for(size_t j=0; j<iit->second.size(); j++)
				res.instances.push_back(iit->second[j]);
		}

		return true;
	}

	bool KnowledgeBase::getInstanceAttr(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getInstanceAttr response for %s %s", req.type_name.c_str(), req.instance_name.c_str());

		// fetch the instances of the correct type
		std::map<std::string,std::vector<rosplan_knowledge_msgs::KnowledgeItem> >::iterator iit;
		iit = instance_attributes.find(req.instance_name);
		if(iit != instance_attributes.end()) {
			// check to make sure each knowledge item is of the correct instance type
			for(size_t j=0; j<iit->second.size(); j++) {
				if(iit->second[j].instance_type.compare(req.type_name)==0) {
					res.attributes.push_back(iit->second[j]);
				}
			}
		}

		return true;
	}

	bool KnowledgeBase::getDomainAttr(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getDomainAttr response for %s", req.predicate_name.c_str());

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<domain_attributes.size(); i++) {
			if(0==req.predicate_name.compare(domain_attributes[i].attribute_name))
				res.attributes.push_back(domain_attributes[i]);
		}

		// ...or fetch the knowledgeItems of the correct function
		for(size_t i=0; i<domain_functions.size(); i++) {
			if(0==req.predicate_name.compare(domain_functions[i].attribute_name))
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
	ros::ServiceServer attributeServer = n.advertiseService("/kcl_rosplan/get_instance_attributes", &KCL_rosplan::KnowledgeBase::getInstanceAttr, &kb);
	ros::ServiceServer domainServer = n.advertiseService("/kcl_rosplan/get_domain_attributes", &KCL_rosplan::KnowledgeBase::getDomainAttr, &kb);
	ros::ServiceServer goalServer = n.advertiseService("/kcl_rosplan/get_current_goals", &KCL_rosplan::KnowledgeBase::getCurrentGoals, &kb);

	// filter services
	kb.notificationPublisher = n.advertise<rosplan_knowledge_msgs::Notification>("/kcl_rosplan/notification", 10, true);
	ros::Subscriber planningFilterSub = n.subscribe("/kcl_rosplan/planning_filter", 100, &KCL_rosplan::KnowledgeBase::planningFilterCallback, &kb);
	ros::Subscriber missionFilterSub = n.subscribe("/kcl_rosplan/mission_filter", 100, &KCL_rosplan::KnowledgeBase::missionFilterCallback, &kb);

	// wait for and clear mongoDB 
	ROS_INFO("KCL: (KB) Waiting for MongoDB");
	ros::service::waitForService("/message_store/delete",-1);
	system("mongo message_store --eval \"printjson(db.message_store.remove())\"");

	ROS_INFO("KCL: (KB) Ready to receive");
	ros::spin();

	return 0;
}
