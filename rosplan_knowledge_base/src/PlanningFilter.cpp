#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_base/KnowledgeBase.h"

namespace KCL_rosplan {

	/**
	 * returns true is the knowledge item contains the instance, as instance or attribute parameter.
	 */
	bool KnowledgeBase::containsInstance(const rosplan_knowledge_msgs::KnowledgeItem &a, std::string &name) {

		if(0==a.instance_name.compare(name))
			return true;

		for(size_t i=0;i<a.values.size();i++) {
			if(0==a.values[i].value.compare(name))
				return true;
		}

		return false;
	}

	/** 
	 * returns true iff a is the same knowledge as b.
	 */
	bool KnowledgeBase::sameKnowledge(const rosplan_knowledge_msgs::KnowledgeItem &a, const rosplan_knowledge_msgs::KnowledgeItem &b) {

		if(a.knowledge_type != b.knowledge_type) return false;
	
		if(a.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {
			if(0!=a.instance_type.compare(b.instance_type)) return false;
			if(0!=a.instance_name.compare(b.instance_name)) return false;
		} else {
			if(0!=a.attribute_name.compare(b.attribute_name)) return false;
			if(a.values.size() != b.values.size()) return false;
			for(size_t i=0;i<a.values.size();i++) {
				if(0!=a.values[i].key.compare(b.values[i].key)) return false;
				if(0!=a.values[i].value.compare(b.values[i].value)) return false;
			}
		}

		return true;
	}

	/** 
	 * returns true iff a matches the knowledge in b.
	 */
	bool KnowledgeBase::containsKnowledge(const rosplan_knowledge_msgs::KnowledgeItem &a, const rosplan_knowledge_msgs::KnowledgeItem &b) {

		if(a.knowledge_type != b.knowledge_type) return false;
	
		if(a.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {
			if(0!=a.instance_type.compare(b.instance_type)) return false;
			if(a.instance_name!="" && 0!=a.instance_name.compare(b.instance_name)) return false;
		} else {
			if(a.attribute_name!="" && 0!=a.attribute_name.compare(b.attribute_name)) return false;
			for(size_t i=0;i<a.values.size();i++) {
				bool match = false;
				for(size_t j=0;j<b.values.size();j++) {
					if(a.values[i].key == b.values[j].key && a.values[i].value == b.values[j].value)
						match = true;
				}
				if(!match) return false;
			}
		}

		return true;
	}

	/** 
	 * returns true iff b is in the filter of a.
	 */
	bool KnowledgeBase::isInFilter(const rosplan_knowledge_msgs::KnowledgeItem &a, const rosplan_knowledge_msgs::KnowledgeItem &b) {

		if(a.knowledge_type != b.knowledge_type) return false;
	
		if(a.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {
			if(0!=a.instance_type.compare(b.instance_type)) return false;
			if(0==a.instance_name.compare("")) return true;
			if(0!=a.instance_name.compare(b.instance_name)) return false;
		} else {
			if(0!=a.attribute_name.compare(b.attribute_name)) return false;
			if(a.values.size() != b.values.size()) return false;
			for(size_t i=0;i<a.values.size();i++) {
				if(0!=a.values[i].key.compare(b.values[i].key)) return false;
				if(0!=a.values[i].value.compare(b.values[i].value)) return false;
			}
		}

		return true;
	}

	/**
	 * check filter
	 */
	void KnowledgeBase::checkFilters(const rosplan_knowledge_msgs::KnowledgeItem &a, bool added) {

		bool filterViolated = false;

		if(added) {
			for(size_t i=0;i<missionFilter.size();i++) {
				if(isInFilter(missionFilter[i], a)) {
					filterViolated = true;
					break;
				}
			}
		}

		if(!added) {
			for(size_t i=0;i<planningFilter.size();i++) {
				if(isInFilter(planningFilter[i], a)) {
					filterViolated = true;
					break;
				}
			}
		}

		if(filterViolated) {
			ROS_INFO("KCL: (KB) Filter violated, sending notification");	

			rosplan_knowledge_msgs::Notification msg;
			if(added) msg.function = rosplan_knowledge_msgs::Notification::ADDED;
			else msg.function = rosplan_knowledge_msgs::Notification::REMOVED;
			msg.knowledge_item = a;
			notificationPublisher.publish(msg);
		}
	}

	/**
	 * Recieve filter from planning_system and store.
	 */
	void KnowledgeBase::planningFilterCallback(const rosplan_knowledge_msgs::Filter::ConstPtr& msg) {
		
		ROS_INFO("KCL: (KB) Filter updated (CLEAR=0;ADD=1;REMOVE=2) function %i", msg->function);

		if(msg->function == rosplan_knowledge_msgs::Filter::CLEAR) {
			planningFilter.clear();
		}

		if(msg->function == rosplan_knowledge_msgs::Filter::ADD) {
			for(size_t i=0; i<msg->knowledge_items.size(); i++)
				planningFilter.push_back(msg->knowledge_items[i]);
		}

		if(msg->function == rosplan_knowledge_msgs::Filter::REMOVE) {
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator position = planningFilter.begin();
			for(size_t i=0; i<msg->knowledge_items.size(); i++) {
				for(; position != planningFilter.end(); position++) {
					rosplan_knowledge_msgs::KnowledgeItem item = msg->knowledge_items[i];
					if(sameKnowledge(item, *position))
						planningFilter.erase(position);
				}
			}
		}
	}

	/**
	 * Recieve mission filter.
	 */
	void KnowledgeBase::missionFilterCallback(const rosplan_knowledge_msgs::Filter::ConstPtr& msg) {
		
		ROS_INFO("KCL: (KB) Mission Filter updated (CLEAR=0;ADD=1;REMOVE=2) function %i", msg->function);

		if(msg->function == rosplan_knowledge_msgs::Filter::CLEAR) {
			missionFilter.clear();
		}

		if(msg->function == rosplan_knowledge_msgs::Filter::ADD) {
			for(size_t i=0; i<msg->knowledge_items.size(); i++)
				missionFilter.push_back(msg->knowledge_items[i]);
		}

		if(msg->function == rosplan_knowledge_msgs::Filter::REMOVE) {
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator position = missionFilter.begin();
			for(size_t i=0; i<msg->knowledge_items.size(); i++) {
				for(; position != missionFilter.end(); position++) {
					rosplan_knowledge_msgs::KnowledgeItem item = msg->knowledge_items[i];
					if(sameKnowledge(item, *position))
						missionFilter.erase(position);
				}
			}
		}
	}
} // close namespace
