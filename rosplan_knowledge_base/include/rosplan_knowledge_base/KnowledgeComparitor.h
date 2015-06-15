#include <ros/ros.h>
#include <vector>
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#ifndef KCL_knowledgecomparitor
#define KCL_knowledgecomparitor

namespace KCL_rosplan {

	class KnowledgeComparitor
	{
	public:
		// checking if filters are violated
		static bool containsKnowledge(const rosplan_knowledge_msgs::KnowledgeItem &a, const rosplan_knowledge_msgs::KnowledgeItem &b);
		static bool containsInstance(const rosplan_knowledge_msgs::KnowledgeItem &a, std::string &name);
	};
}
#endif
