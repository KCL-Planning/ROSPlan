#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "KnowledgeBase.h"

namespace KCL_rosplan {

	/*--------------*/
	/* adding items */
	/*--------------*/

	void KnowledgeBase::addInstance(const planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg) {
		ROS_INFO("KCL: (KB) Adding instance (%s, %s)", msg->instance_type.c_str(), msg->instance_name.c_str());
		domainInstances[msg->instance_type].push_back(msg->instance_name);
	}

	void KnowledgeBase::addKnowledge(const planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg) {
		
		if(msg->knowledge_type == planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE) {
			ROS_INFO("KCL: (KB) Adding domain attribute (%s)", msg->attribute_name.c_str());
			domainAttributes.push_back(*msg);
		}

		if(msg->knowledge_type == planning_knowledge_msgs::KnowledgeItem::INSTANCE_ATTRIBUTE) {
			ROS_INFO("KCL: (KB) Adding instance attribute (%s, %s)", msg->instance_name.c_str(), msg->attribute_name.c_str());
			instanceAttributes[msg->instance_name].push_back(*msg);
		}
	}

	void KnowledgeBase::addMissionGoal(const planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg) {
		ROS_INFO("KCL: (KB) Adding mission goal (%s)", msg->attribute_name.c_str());
		domainGoals.push_back(*msg);
	}

	/*----------------*/
	/* fetching items */
	/*----------------*/

	bool KnowledgeBase::getInstances(planning_knowledge_msgs::InstanceService::Request  &req, planning_knowledge_msgs::InstanceService::Response &res)
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

	bool KnowledgeBase::getInstanceAttr(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getInstanceAttr response for %s %s", req.type_name.c_str(), req.instance_name.c_str());

		// fetch the instances of the correct type
		std::map<std::string,std::vector<planning_knowledge_msgs::KnowledgeItem> >::iterator iit;
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

	bool KnowledgeBase::getDomainAttr(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getDomainAttr response for %s", req.predicate_name.c_str());

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<domainAttributes.size(); i++) {
			if(req.predicate_name.compare(domainAttributes[i].attribute_name)==0)
				res.attributes.push_back(domainAttributes[i]);
		}

		return true;
	}

	bool KnowledgeBase::getCurrentGoals(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res)
	{
		ROS_INFO("KCL: (KB) Sending getCurrentGoals response");

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<domainGoals.size(); i++) {
			res.attributes.push_back(domainGoals[i]);
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

	// TESTING for SQUIRREL summer school
	{
		// objects
		kb.domainInstances["room"].push_back("room");
		kb.domainInstances["object"].push_back("bananna");

		// goals
		planning_knowledge_msgs::KnowledgeItem unexplored;
		unexplored.knowledge_type = planning_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE;
		unexplored.attribute_name = "explored";
		diagnostic_msgs::KeyValue room;
		room.key = "r";
		room.value = "room";
		unexplored.values.push_back(room);
		kb.domainGoals.push_back(unexplored);

		// mission filter
		planning_knowledge_msgs::KnowledgeItem toyFilter;
		toyFilter.knowledge_type = planning_knowledge_msgs::KnowledgeItem::INSTANCE;
		toyFilter.instance_type = "toy";
		toyFilter.instance_name = "";
		kb.missionFilter.push_back(toyFilter);
	}
	// END TESTING

	// update environment services
	kb.instanceServer = n.advertiseService("/kcl_rosplan/get_instances", &KCL_rosplan::KnowledgeBase::getInstances, &kb);
	kb.attributeServer = n.advertiseService("/kcl_rosplan/get_instance_attributes", &KCL_rosplan::KnowledgeBase::getInstanceAttr, &kb);
	kb.domainServer = n.advertiseService("/kcl_rosplan/get_domain_attributes", &KCL_rosplan::KnowledgeBase::getDomainAttr, &kb);
	kb.goalServer = n.advertiseService("/kcl_rosplan/get_current_goals", &KCL_rosplan::KnowledgeBase::getCurrentGoals, &kb);

	// filter services
	kb.notificationPublisher = n.advertise<planning_knowledge_msgs::Notification>("/kcl_rosplan/notification", 10, true);
	ros::Subscriber filterSub = n.subscribe("/kcl_rosplan/filter", 100, &KCL_rosplan::KnowledgeBase::planningFilterCallback, &kb);

	// add item subscribers
	ros::Subscriber addInstanceSub = n.subscribe("/kcl_rosplan/add_instance", 100, &KCL_rosplan::KnowledgeBase::addInstance, &kb);
	ros::Subscriber addAttrSub = n.subscribe("/kcl_rosplan/add_knowledge_item", 100, &KCL_rosplan::KnowledgeBase::addKnowledge, &kb);
	ros::Subscriber addMissionGoalSub = n.subscribe("/kcl_rosplan/add_mission_goal", 100, &KCL_rosplan::KnowledgeBase::addMissionGoal, &kb);

	ROS_INFO("KCL: (KB) Ready to receive");
	ros::spin();

	return 0;
}
