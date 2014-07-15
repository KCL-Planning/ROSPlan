#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "planning_knowledge_msgs/InstanceService.h"
#include "planning_knowledge_msgs/AttributeService.h"
#include "planning_knowledge_msgs/KnowledgeItem.h"
#include "planning_knowledge_msgs/Notification.h"
#include "planning_knowledge_msgs/Filter.h"

#ifndef KCL_knowledgebase
#define KCL_knowledgebase

namespace KCL_rosplan {

	class KnowledgeBase
	{
	private:

		bool sameKnowledge(const planning_knowledge_msgs::KnowledgeItem &a, const planning_knowledge_msgs::KnowledgeItem &b);

	public:

		// symbolic model
		std::map<std::string, std::vector<std::string> > domainInstances;
		std::vector<planning_knowledge_msgs::KnowledgeItem> domainAttributes;
		std::vector<planning_knowledge_msgs::KnowledgeItem> domainFunctions;
		std::vector<planning_knowledge_msgs::KnowledgeItem> domainGoals;
		std::map<std::string, std::vector<planning_knowledge_msgs::KnowledgeItem> > instanceAttributes;

		// planning filter
		std::vector<planning_knowledge_msgs::KnowledgeItem> planningFilter;
		std::vector<planning_knowledge_msgs::KnowledgeItem> missionFilter;

		// topic methods for planning_system notification
		void planningFilterCallback(const planning_knowledge_msgs::Filter::ConstPtr& msg);
		ros::Publisher notificationPublisher;

		// service methods for fetching the symbolic model
		ros::ServiceServer attributeServer;
		ros::ServiceServer instanceServer;
		ros::ServiceServer domainServer;
		ros::ServiceServer goalServer;

		bool getInstances(planning_knowledge_msgs::InstanceService::Request  &req, planning_knowledge_msgs::InstanceService::Response &res);
		bool getInstanceAttr(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res);
		bool getDomainAttr(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res);
		bool getCurrentGoals(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res);

		// adding items to the knowledge base
		void addInstance(const planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
		void addKnowledge(const planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
		void addMissionGoal(const planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
	};
}
#endif
