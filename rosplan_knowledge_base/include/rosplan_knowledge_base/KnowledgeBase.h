#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
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

		// adding and removing items to and from the knowledge base
		void addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
		void addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);
		void removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);

	public:

		// symbolic model
		std::map<std::string, std::vector<std::string> > domain_instances;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> domain_attributes;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> domain_functions;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> domain_goals;
		std::map<std::string, std::vector<rosplan_knowledge_msgs::KnowledgeItem> > instance_attributes;

		// fetching the symbolic model
		bool getInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res);
		bool getInstanceAttr(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
		bool getDomainAttr(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
		bool getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);

		// adding and removing items to and from the knowledge base
		bool updateKnowledge(rosplan_knowledge_msgs::KnowledgeUpdateService::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateService::Response &res);

		// planning and mission filter
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> planningFilter;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> missionFilter;

		// planning_system notification
		void planningFilterCallback(const rosplan_knowledge_msgs::Filter::ConstPtr& msg);
		ros::Publisher notificationPublisher;
	};
}
#endif
