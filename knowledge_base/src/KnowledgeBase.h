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

		// checking if filters are violated
		bool containsInstance(const planning_knowledge_msgs::KnowledgeItem &a, std::string &name);
		bool sameKnowledge(const planning_knowledge_msgs::KnowledgeItem &a, const planning_knowledge_msgs::KnowledgeItem &b);
		bool isInFilter(const planning_knowledge_msgs::KnowledgeItem &a, const planning_knowledge_msgs::KnowledgeItem &b);
		void checkFilters(const planning_knowledge_msgs::KnowledgeItem &a, bool added);

	public:

		// symbolic model
		std::map<std::string, std::vector<std::string> > domainInstances;
		std::vector<planning_knowledge_msgs::KnowledgeItem> domainAttributes;
		std::vector<planning_knowledge_msgs::KnowledgeItem> domainFunctions;
		std::vector<planning_knowledge_msgs::KnowledgeItem> domainGoals;
		std::map<std::string, std::vector<planning_knowledge_msgs::KnowledgeItem> > instanceAttributes;

		// planning and mission filter
		std::vector<planning_knowledge_msgs::KnowledgeItem> planningFilter;
		std::vector<planning_knowledge_msgs::KnowledgeItem> missionFilter;

		// planning_system notification
		void planningFilterCallback(const planning_knowledge_msgs::Filter::ConstPtr& msg);
		ros::Publisher notificationPublisher;

		// fetching the symbolic model
		bool getInstances(planning_knowledge_msgs::InstanceService::Request  &req, planning_knowledge_msgs::InstanceService::Response &res);
		bool getInstanceAttr(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res);
		bool getDomainAttr(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res);
		bool getCurrentGoals(planning_knowledge_msgs::AttributeService::Request  &req, planning_knowledge_msgs::AttributeService::Response &res);

		// adding and removing items to and from the knowledge base
		void addKnowledge(const planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
		void addMissionGoal(const planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
		void removeKnowledge(const planning_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
	};
}
#endif
