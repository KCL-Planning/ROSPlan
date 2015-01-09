#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "rosplan_knowledge_msgs/InstanceService.h"
#include "rosplan_knowledge_msgs/AttributeService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"

#ifndef KCL_knowledgebase
#define KCL_knowledgebase

namespace KCL_rosplan {

	class KnowledgeBase
	{
	private:

		// checking if filters are violated
		bool containsInstance(const rosplan_knowledge_msgs::KnowledgeItem &a, std::string &name);
		bool sameKnowledge(const rosplan_knowledge_msgs::KnowledgeItem &a, const rosplan_knowledge_msgs::KnowledgeItem &b);
		bool isInFilter(const rosplan_knowledge_msgs::KnowledgeItem &a, const rosplan_knowledge_msgs::KnowledgeItem &b);
		void checkFilters(const rosplan_knowledge_msgs::KnowledgeItem &a, bool added);

	public:

		// symbolic model
		std::map<std::string, std::vector<std::string> > domainInstances;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> domainAttributes;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> domainFunctions;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> domainGoals;
		std::map<std::string, std::vector<rosplan_knowledge_msgs::KnowledgeItem> > instanceAttributes;

		// fetching the symbolic model
		bool getInstances(rosplan_knowledge_msgs::InstanceService::Request  &req, rosplan_knowledge_msgs::InstanceService::Response &res);
		bool getInstanceAttr(rosplan_knowledge_msgs::AttributeService::Request  &req, rosplan_knowledge_msgs::AttributeService::Response &res);
		bool getDomainAttr(rosplan_knowledge_msgs::AttributeService::Request  &req, rosplan_knowledge_msgs::AttributeService::Response &res);
		bool getCurrentGoals(rosplan_knowledge_msgs::AttributeService::Request  &req, rosplan_knowledge_msgs::AttributeService::Response &res);

		// adding and removing items to and from the knowledge base
		void addKnowledge(const rosplan_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
		void addMissionGoal(const rosplan_knowledge_msgs::KnowledgeItem::ConstPtr& msg);
		void removeKnowledge(const rosplan_knowledge_msgs::KnowledgeItem::ConstPtr& msg);

		// planning and mission filter
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> planningFilter;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> missionFilter;

		// planning_system notification
		void planningFilterCallback(const rosplan_knowledge_msgs::Filter::ConstPtr& msg);
		ros::Publisher notificationPublisher;
	};
}
#endif
