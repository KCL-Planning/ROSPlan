#include "rosplan_knowledge_base/KnowledgeComparitor.h"

/* implementation of KnowledgeComparitor.h */
namespace KCL_rosplan {

	/** 
	 * returns true iff a matches the knowledge in b.
	 */
	bool KnowledgeComparitor::containsKnowledge(const rosplan_knowledge_msgs::KnowledgeItem &a, const rosplan_knowledge_msgs::KnowledgeItem &b) {

		if(a.knowledge_type != b.knowledge_type) return false;
	
		if(a.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {
			
			// check instance knowledge
			if(0!=a.instance_type.compare(b.instance_type)) return false;
			if(a.instance_name!="" && 0!=a.instance_name.compare(b.instance_name)) return false;

		} else {

			// check fact or function
			if(a.attribute_name!="" && 0!=a.attribute_name.compare(b.attribute_name)) return false;
			if(a.is_negative != b.is_negative) return false;
			if(a.values.size() != b.values.size()) return false;
			for(size_t i=0;i<a.values.size();i++) {
				if(""!=a.values[i].value && a.values[i].value != b.values[i].value)
					return false;
			}
		}

		return true;
	}

	/**
	 * returns true is the knowledge item contains the instance, as instance or attribute parameter.
	 */
	bool KnowledgeComparitor::containsInstance(const rosplan_knowledge_msgs::KnowledgeItem &a, std::string &name) {

		if(0==a.instance_name.compare(name))
			return true;

		for(size_t i=0;i<a.values.size();i++) {
			if(0==a.values[i].value.compare(name))
				return true;
		}

		return false;
	}

} // close namespace
