#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"

#include "KnowledgeComparitor.h"

#ifndef KCL_knowledgeplanfilter
#define KCL_knowledgeplanfilter

namespace KCL_rosplan {

	class PlanFilter
	{
	public:

		// planning and mission filter
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> planning_filter;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> mission_filter;
		void checkFilters(const rosplan_knowledge_msgs::KnowledgeItem &a, bool added);

		// planning_system notification
		void planningFilterCallback(const rosplan_knowledge_msgs::Filter::ConstPtr& msg);
		void missionFilterCallback(const rosplan_knowledge_msgs::Filter::ConstPtr& msg);
		ros::Publisher notification_publisher;
	};
}
#endif
