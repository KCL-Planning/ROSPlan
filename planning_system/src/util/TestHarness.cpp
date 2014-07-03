#include <ros/ros.h>
#include <vector>
#include "planning_knowledge_msgs/InstanceService.h"
#include "planning_knowledge_msgs/AttributeService.h"
#include "planning_knowledge_msgs/KnowledgeItem.h"
#include <iostream>
#include <fstream>

bool getInstances(planning_knowledge_msgs::InstanceService::Request  &req, planning_knowledge_msgs::InstanceService::Response &res)
{

	ros::NodeHandle n;
	ROS_INFO("Sending getInstances.");
	
	if(req.type_name.compare("gripper")==0) { 
		res.instances.push_back("g1");
	}

	if(req.type_name.compare("block")==0) {
		res.instances.push_back("b1");
		res.instances.push_back("b2");
		res.instances.push_back("b3");
	}

	return true;
}

bool getInstanceAttr(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res)
{

	ros::NodeHandle n;

	ROS_INFO("Sending getInstanceAttr response.");

	if(req.type_name.compare("gripper")==0) {
		planning_knowledge_msgs::KnowledgeItem attr;
		attr.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
		attr.instance_type = req.type_name;
		attr.instance_name = req.instance_name;
		attr.attribute_name = "pose";
		diagnostic_msgs::KeyValue pair_N;
		pair_N.key = "N"; pair_N.value = "4.28743";
		diagnostic_msgs::KeyValue pair_E;
		pair_E.key = "E"; pair_E.value = "4.92874";
		diagnostic_msgs::KeyValue pair_D;
		pair_D.key = "D"; pair_D.value = "2.59276";
		attr.values.push_back(pair_N);
		attr.values.push_back(pair_E);
		attr.values.push_back(pair_D);
		res.attributes.push_back(attr);
	}
	return true;
}

bool getDomainAttr(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res)
{

	ros::NodeHandle n;

	ROS_INFO("Sending getDomainAttr response.");

	if(req.predicate_name.compare("empty")==0) {
		planning_knowledge_msgs::KnowledgeItem attr;
		attr.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
		attr.instance_type = "";
		attr.instance_name = "";
		attr.attribute_name = "empty";
		diagnostic_msgs::KeyValue pair;
		pair.key = "g";
		pair.value = "g1";
		attr.values.push_back(pair);
		res.attributes.push_back(attr);
	}

	if(req.predicate_name.compare("onfloor")==0) {
		const std::string blocks[] = {"b1", "b2", "b3"};
		for(int i=0;i<3;i++) {
			planning_knowledge_msgs::KnowledgeItem attr;
			attr.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr.instance_type = "";
			attr.instance_name = "";
			attr.attribute_name = "onfloor";
			diagnostic_msgs::KeyValue pair;
			pair.key = "b";
			pair.value = blocks[i];
			attr.values.push_back(pair);
			res.attributes.push_back(attr);
		}
	}

	if(req.predicate_name.compare("clear")==0) {
		const std::string blocks[] = {"b1", "b2", "b3"};
		for(int i=0;i<3;i++) {
			planning_knowledge_msgs::KnowledgeItem attr;
			attr.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr.instance_type = "";
			attr.instance_name = "";
			attr.attribute_name = "clear";
			diagnostic_msgs::KeyValue pair;
			pair.key = "b";
			pair.value = blocks[i];
			attr.values.push_back(pair);
			res.attributes.push_back(attr);
		}
	}

	if(req.predicate_name.compare("weight")==0) {
		const std::string blocks[] = {"b1", "b2", "b3"};
		for(int i=0;i<3;i++) {
			planning_knowledge_msgs::KnowledgeItem attr;
			attr.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr.instance_type = "";
			attr.instance_name = "";
			attr.attribute_name = "weight";
			diagnostic_msgs::KeyValue pair2;
			pair2.key = "b";
			pair2.value = blocks[i];
			diagnostic_msgs::KeyValue pair3;
			pair3.key = "function_value";
			pair3.value = "1";
			attr.values.push_back(pair2);
			attr.values.push_back(pair3);
			res.attributes.push_back(attr);
		}
	}

	return true;
}

bool getCurrentGoals(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res)
{

	ros::NodeHandle n;

	ROS_INFO("Sending getCurrentGoals response.");

	if(req.instance_name.compare("b1")==0) {

		{ // predicate attributes
			planning_knowledge_msgs::KnowledgeItem attr_on1;
			attr_on1.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr_on1.instance_type = "";
			attr_on1.instance_name = "";
			attr_on1.attribute_name = "on";
			diagnostic_msgs::KeyValue pair_top;
			pair_top.key = "b1"; pair_top.value = "b1";
			attr_on1.values.push_back(pair_top);
			diagnostic_msgs::KeyValue pair_bottom;
			pair_bottom.key = "b2"; pair_bottom.value = "b2";
			attr_on1.values.push_back(pair_bottom);
			res.attributes.push_back(attr_on1);
		}
	}

	if(req.instance_name.compare("b2")==0) {

		{ // predicate attributes
			planning_knowledge_msgs::KnowledgeItem attr_on1;
			attr_on1.knowledge_type = planning_knowledge_msgs::KnowledgeItem::ATTRIBUTE;
			attr_on1.instance_type = "";
			attr_on1.instance_name = "";
			attr_on1.attribute_name = "on";
			diagnostic_msgs::KeyValue pair_top;
			pair_top.key = "b1"; pair_top.value = "b2";
			attr_on1.values.push_back(pair_top);
			diagnostic_msgs::KeyValue pair_bottom;
			pair_bottom.key = "b2"; pair_bottom.value = "b3";
			attr_on1.values.push_back(pair_bottom);
			res.attributes.push_back(attr_on1);
		}
	}

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "KCL_rosplan_harness");
	ros::NodeHandle n;
	ros::ServiceServer service3 = n.advertiseService("/kcl_rosplan/get_instances", getInstances);
	ros::ServiceServer service4 = n.advertiseService("/kcl_rosplan/get_instance_attributes", getInstanceAttr);
	ros::ServiceServer service5 = n.advertiseService("/kcl_rosplan/get_domain_attributes", getDomainAttr);
	ros::ServiceServer service6 = n.advertiseService("/kcl_rosplan/get_current_goals", getCurrentGoals);

	ROS_INFO("Ready to receive.");
	ros::spin();

	return 0;
}
