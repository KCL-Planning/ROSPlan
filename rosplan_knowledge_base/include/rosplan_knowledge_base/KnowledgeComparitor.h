#include <ros/ros.h>
#include <vector>
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/DomainInequality.h"
#include "rosplan_knowledge_msgs/ExprBase.h"
#include "rosplan_knowledge_msgs/ExprComposite.h"

#ifndef KCL_knowledgecomparitor
#define KCL_knowledgecomparitor

namespace KCL_rosplan {

	class KnowledgeComparitor
	{
	public:
		static double getValue(const rosplan_knowledge_msgs::ExprBase &expr, const std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFunctions);
		static double evaluateOperation(const double &lhs, const double &rhs, const rosplan_knowledge_msgs::ExprBase &op);
		static double evaluateExpression(const rosplan_knowledge_msgs::ExprComposite &a, const std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFunctions);
		static bool inequalityTrue(const rosplan_knowledge_msgs::KnowledgeItem &a, const std::vector<rosplan_knowledge_msgs::KnowledgeItem> &modelFunctions);
		static bool containsKnowledge(const rosplan_knowledge_msgs::KnowledgeItem &a, const rosplan_knowledge_msgs::KnowledgeItem &b);
		static bool containsInstance(const rosplan_knowledge_msgs::KnowledgeItem &a, std::string &name);
	};
}
#endif
