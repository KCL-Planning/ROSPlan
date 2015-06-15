#include "rosplan_knowledge_base/PlanFilter.h"

/* implementation of PlanFilter.h */
namespace KCL_rosplan {

	/**
	 * check filter
	 */
	void PlanFilter::checkFilters(const rosplan_knowledge_msgs::KnowledgeItem &a, bool added) {

		bool filterViolated = false;

		if(added) {
			for(size_t i=0;i<mission_filter.size();i++)
				if(KnowledgeComparitor::containsKnowledge(mission_filter[i], a)) {
					filterViolated = true;
					break;
				}
		} else {
			for(size_t i=0;i<planning_filter.size();i++)
				if(KnowledgeComparitor::containsKnowledge(planning_filter[i], a)) {
					filterViolated = true;
					break;
				}
		}

		if(filterViolated) {
			ROS_INFO("KCL: (KB) Filter violated, sending notification");	
			rosplan_knowledge_msgs::Notification msg;
			if(added) msg.function = rosplan_knowledge_msgs::Notification::ADDED;
			else msg.function = rosplan_knowledge_msgs::Notification::REMOVED;
			msg.knowledge_item = a;
			notification_publisher.publish(msg);
		}
	}

	/**
	 * update planning filter callback
	 */
	void PlanFilter::planningFilterCallback(const rosplan_knowledge_msgs::Filter::ConstPtr& msg) {
		
		ROS_INFO("KCL: (KB) Filter updated (CLEAR=0;ADD=1;REMOVE=2) function %i", msg->function);

		if(msg->function == rosplan_knowledge_msgs::Filter::CLEAR) {
			planning_filter.clear();
		}

		if(msg->function == rosplan_knowledge_msgs::Filter::ADD) {
			for(size_t i=0; i<msg->knowledge_items.size(); i++)
				planning_filter.push_back(msg->knowledge_items[i]);
		}

		if(msg->function == rosplan_knowledge_msgs::Filter::REMOVE) {
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator position = planning_filter.begin();
			for(size_t i=0; i<msg->knowledge_items.size(); i++) {
				for(; position != planning_filter.end(); position++) {
					rosplan_knowledge_msgs::KnowledgeItem item = msg->knowledge_items[i];
					if(KnowledgeComparitor::containsKnowledge(item, *position)) {
						position = planning_filter.erase(position);
						if(position!=planning_filter.begin()) position--;
						if(position==planning_filter.end()) break;
					}
				}
			}
		}
	}

	/**
	 * update mission filter callback
	 */
	void PlanFilter::missionFilterCallback(const rosplan_knowledge_msgs::Filter::ConstPtr& msg) {
		
		ROS_INFO("KCL: (KB) Mission Filter updated (CLEAR=0;ADD=1;REMOVE=2) function %i", msg->function);

		if(msg->function == rosplan_knowledge_msgs::Filter::CLEAR) {
			mission_filter.clear();
		}

		if(msg->function == rosplan_knowledge_msgs::Filter::ADD) {
			for(size_t i=0; i<msg->knowledge_items.size(); i++)
				mission_filter.push_back(msg->knowledge_items[i]);
		}

		if(msg->function == rosplan_knowledge_msgs::Filter::REMOVE) {
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator position = mission_filter.begin();
			for(size_t i=0; i<msg->knowledge_items.size(); i++) {
				for(; position != mission_filter.end(); position++) {
					rosplan_knowledge_msgs::KnowledgeItem item = msg->knowledge_items[i];
					if(KnowledgeComparitor::containsKnowledge(item, *position)) {
						position = mission_filter.erase(position);
						if(position!=mission_filter.begin()) position--;
						if(position==mission_filter.end()) break;
					}
				}
			}
		}
	}
} // close namespace
