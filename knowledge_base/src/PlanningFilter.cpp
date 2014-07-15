#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include "planning_knowledge_msgs/Notification.h"
#include "planning_knowledge_msgs/Filter.h"
#include "planning_knowledge_msgs/KnowledgeItem.h"
#include "KnowledgeBase.h"

namespace KCL_rosplan {

	/** 
	 * returns true iff the two knowledge items contain exactly the same knowledge.
	 */
	bool KnowledgeBase::sameKnowledge(const planning_knowledge_msgs::KnowledgeItem &a, const planning_knowledge_msgs::KnowledgeItem &b) {

		if(a.knowledge_type != b.knowledge_type) return false;
	
		if(a.knowledge_type == planning_knowledge_msgs::KnowledgeItem::INSTANCE) {
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
	 * Recieve filter from planning_system and store.
	 */
	void KnowledgeBase::planningFilterCallback(const planning_knowledge_msgs::Filter::ConstPtr& msg) {
		
		ROS_INFO("KCL: (KB) Filter updated (CLEAR=0;ADD=1;REMOVE=2) function %i", msg->function);

		if(msg->function == planning_knowledge_msgs::Filter::CLEAR) {
			planningFilter.clear();
		}

		if(msg->function == planning_knowledge_msgs::Filter::ADD) {
			for(size_t i=0; i<msg->knowledge_items.size(); i++)
				planningFilter.push_back(msg->knowledge_items[i]);
		}

		if(msg->function == planning_knowledge_msgs::Filter::REMOVE) {
			std::vector<planning_knowledge_msgs::KnowledgeItem>::iterator position = planningFilter.begin();
			for(size_t i=0; i<msg->knowledge_items.size(); i++) {
				for(; position != planningFilter.end(); position++) {
					planning_knowledge_msgs::KnowledgeItem item = msg->knowledge_items[i];
					if(sameKnowledge(item, *position))
						planningFilter.erase(position);
				}
			}
		}
	}
} // close namespace
