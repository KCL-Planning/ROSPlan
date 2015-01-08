#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "rosplan_knowledge_base/KnowledgeBase.h"

namespace KCL_rosplan {

	/*----------------*/
	/* removing items */
	/*----------------*/

	void KnowledgeBase::removeKnowledge(const rosplan_knowledge_msgs::KnowledgeItem::ConstPtr& msg) {

		if(msg->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {		

			// search for instance
			std::vector<std::string>::iterator iit;
			iit = domainInstances[msg->instance_type].begin();
			while(iit!=domainInstances[msg->instance_type].end()) {
				std::string name = *iit;
				if(name.compare(msg->instance_name)==0 || msg->instance_name.compare("")==0) {

					// remove instance from knowledge base
					ROS_INFO("KCL: (KB) Removing instance (%s, %s)", msg->instance_type.c_str(), msg->instance_name.c_str());
					checkFilters(*msg, false);
					domainInstances[msg->instance_type].erase(iit);

					// remove affected domain attributes
					std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
					for(pit=domainAttributes.begin(); pit!=domainAttributes.end(); pit++) {
						if(containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing domain attribute (%s)", pit->attribute_name.c_str());
							checkFilters(*pit, false);
							pit = domainAttributes.erase(pit);
							if(pit==domainAttributes.end()) break;
						}
					}

					// remove affected instance attributes
					for(pit=instanceAttributes[name].begin(); pit!=instanceAttributes[name].end(); pit++) {
						if(containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing instance attribute (%s, %s)", name.c_str(), pit->attribute_name.c_str());
							checkFilters(*pit, false);
							pit = instanceAttributes[name].erase(pit);
							if(pit==instanceAttributes[name].end()) break;
						}
					}

					// stop looping through instances
					break;
				} 
			}
		} else if(msg->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

			// remove domain attribute (predicate) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domainAttributes.begin(); pit!=domainAttributes.end(); pit++) {
				if(sameKnowledge(*msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg->attribute_name.c_str());
					checkFilters(*msg, false);
					pit = domainAttributes.erase(pit);
					if(pit==domainAttributes.end()) break;
				}
			}

		} else if(msg->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE_ATTRIBUTE) {

			// remove instance attribute (non-symbolic) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=instanceAttributes[msg->instance_name].begin(); pit!=instanceAttributes[msg->instance_name].end(); pit++) {
				if(sameKnowledge(*msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing instance attribute (%s, %s)", msg->instance_name.c_str(), msg->attribute_name.c_str());
					checkFilters(*msg, false);
					pit = instanceAttributes[msg->instance_name].erase(pit);
					if(pit==instanceAttributes[msg->instance_name].end()) break;
				}
			}
		}
	}

	/*--------------*/
	/* adding items */
	/*--------------*/

	void KnowledgeBase::addKnowledge(const rosplan_knowledge_msgs::KnowledgeItem::ConstPtr& msg) {
		
		if(msg->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {

			// check if instance is already in knowledge base
			std::vector<std::string>::iterator iit;
			iit = find(domainInstances[msg->instance_type].begin(), domainInstances[msg->instance_type].end(), msg->instance_name);

			if(iit==domainInstances[msg->instance_type].end()) {
				// add instance to knowledge base
				ROS_INFO("KCL: (KB) Adding instance (%s, %s)", msg->instance_type.c_str(), msg->instance_name.c_str());
				domainInstances[msg->instance_type].push_back(msg->instance_name);
				checkFilters(*msg, true);
			}

		} else if(msg->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {

			// add domain attribute to knowledge base		
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=domainAttributes.begin(); pit!=domainAttributes.end(); pit++) {
				if(sameKnowledge(*msg, *pit)) {
					// already added
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding domain attribute (%s)", msg->attribute_name.c_str());
			domainAttributes.push_back(*msg);
			checkFilters(*msg, true);

		} else if(msg->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE_ATTRIBUTE) {

			// add instance attribute to knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=instanceAttributes[msg->instance_name].begin(); pit!=instanceAttributes[msg->instance_name].end(); pit++) {
				if(sameKnowledge(*msg, *pit)) {
					// already added
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding instance attribute (%s, %s)", msg->instance_name.c_str(), msg->attribute_name.c_str());
			instanceAttributes[msg->instance_name].push_back(*msg);
			checkFilters(*msg, true);
		}
	}

	void KnowledgeBase::addMissionGoal(const rosplan_knowledge_msgs::KnowledgeItem::ConstPtr& msg) {
		// add mission goal to knowledge base
		ROS_INFO("KCL: (KB) Adding mission goal (%s)", msg->attribute_name.c_str());
		domainGoals.push_back(*msg);
	}

	/*----------------*/
	/* fetching items */
	/*----------------*/

	bool KnowledgeBase::getInstances(rosplan_knowledge_msgs::InstanceService::Request  &req, rosplan_knowledge_msgs::InstanceService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending %s getInstances", req.type_name.c_str());
	
		// fetch the instances of the correct type
		std::map<std::string,std::vector<std::string> >::iterator iit;
		iit = domainInstances.find(req.type_name);
		if(iit != domainInstances.end()) {
			for(size_t j=0; j<iit->second.size(); j++)
				res.instances.push_back(iit->second[j]);
		}

		return true;
	}

	bool KnowledgeBase::getInstanceAttr(rosplan_knowledge_msgs::AttributeService::Request  &req, rosplan_knowledge_msgs::AttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getInstanceAttr response for %s %s", req.type_name.c_str(), req.instance_name.c_str());

		// fetch the instances of the correct type
		std::map<std::string,std::vector<rosplan_knowledge_msgs::KnowledgeItem> >::iterator iit;
		iit = instanceAttributes.find(req.instance_name);
		if(iit != instanceAttributes.end()) {
			// check to make sure each knowledge item is of the correct instance type
			for(size_t j=0; j<iit->second.size(); j++) {
				if(iit->second[j].instance_type.compare(req.type_name)==0) {
					res.attributes.push_back(iit->second[j]);
				}
			}
		}

		return true;
	}

	bool KnowledgeBase::getDomainAttr(rosplan_knowledge_msgs::AttributeService::Request  &req, rosplan_knowledge_msgs::AttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getDomainAttr response for %s", req.predicate_name.c_str());

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<domainAttributes.size(); i++) {
			if(req.predicate_name.compare(domainAttributes[i].attribute_name)==0)
				res.attributes.push_back(domainAttributes[i]);
		}

		// ...or fetch the knowledgeItems of the correct function
		for(size_t i=0; i<domainFunctions.size(); i++) {
			if(req.predicate_name.compare(domainFunctions[i].attribute_name)==0)
				res.attributes.push_back(domainFunctions[i]);
		}

		return true;
	}

	bool KnowledgeBase::getCurrentGoals(rosplan_knowledge_msgs::AttributeService::Request  &req, rosplan_knowledge_msgs::AttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getCurrentGoals response");

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<domainGoals.size(); i++) {
			res.attributes.push_back(domainGoals[i]);
		}

		/* TESTING for SQUIRREL integration meeting 1/2
		...
		// END TESTING */

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

	/* TESTING for SQUIRREL integration meeting 2/2
	...
	// END TESTING */

	// environment services
	ros::ServiceServer instanceServer = n.advertiseService("/kcl_rosplan/get_instances", &KCL_rosplan::KnowledgeBase::getInstances, &kb);
	ros::ServiceServer attributeServer = n.advertiseService("/kcl_rosplan/get_instance_attributes", &KCL_rosplan::KnowledgeBase::getInstanceAttr, &kb);
	ros::ServiceServer domainServer = n.advertiseService("/kcl_rosplan/get_domain_attributes", &KCL_rosplan::KnowledgeBase::getDomainAttr, &kb);
	ros::ServiceServer goalServer = n.advertiseService("/kcl_rosplan/get_current_goals", &KCL_rosplan::KnowledgeBase::getCurrentGoals, &kb);

	// filter services
	kb.notificationPublisher = n.advertise<rosplan_knowledge_msgs::Notification>("/kcl_rosplan/notification", 10, true);
	ros::Subscriber filterSub = n.subscribe("/kcl_rosplan/filter", 100, &KCL_rosplan::KnowledgeBase::planningFilterCallback, &kb);

	// knowledge subscribers
	ros::Subscriber addKnowledgeSub = n.subscribe("/kcl_rosplan/add_knowledge", 100, &KCL_rosplan::KnowledgeBase::addKnowledge, &kb);
	ros::Subscriber removeKnowledgeSub = n.subscribe("/kcl_rosplan/remove_knowledge", 100, &KCL_rosplan::KnowledgeBase::removeKnowledge, &kb);
	ros::Subscriber addMissionGoalSub = n.subscribe("/kcl_rosplan/add_mission_goal", 100, &KCL_rosplan::KnowledgeBase::addMissionGoal, &kb);

	ROS_INFO("KCL: (KB) Ready to receive");
	ros::spin();

	return 0;
}
