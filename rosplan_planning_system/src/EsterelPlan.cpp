#include "rosplan_planning_system/EsterelPlan.h"

#include <vector>
#include <string>
#include <ostream>

std::ostream& operator<<(std::ostream& os, const KCL_rosplan::StrlEdge& edge)
{
	os << "\t[Esterel Edge] - ";
	switch (edge.signal_type)
	{
		case KCL_rosplan::ACTION:
			os << "Action: ";
			break;
		case KCL_rosplan::CONDITION:
			os << "Condition: ";
			break;
		case KCL_rosplan::TIME:
			os << "Time: ";
			break;
	};
	
	os << edge.edge_name << (edge.active ? " ACTIVE" : " INACTIVE") << "; Addr: " << &edge << std::endl;
	os << "\t\tSources: " << std::endl;
	for (std::vector<KCL_rosplan::StrlNode*>::const_iterator ci = edge.sources.begin(); ci != edge.sources.end(); ++ci)
	{
		os << "\t\t\t" << (*ci)->node_name << std::endl;
	}
	
	os << "\t\tSinks: " << std::endl;
	for (std::vector<KCL_rosplan::StrlNode*>::const_iterator ci = edge.sinks.begin(); ci != edge.sinks.end(); ++ci)
	{
		os << "\t\t\t" << (*ci)->node_name << std::endl;
	}
	
	os << "\t\tExternal conditions: " << std::endl;
	for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = edge.external_conditions.begin(); ci != edge.external_conditions.end(); ++ci)
	{
		const rosplan_knowledge_msgs::KnowledgeItem& ki = *ci;
		os << "\t\t\t";
		if (ki.is_negative)
		{
			os << "(not ";
		}
		os << "(" << ki.attribute_name << " ";
		for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = ki.values.begin(); ci != ki.values.end(); ++ci)
		{
			os << (*ci).value << " ";
		}
		if (ki.is_negative)
		{
			os << ")";
		}
		os << ")" << std::endl;
	}
	return os;
};

	
std::ostream& operator<<(std::ostream& os, const KCL_rosplan::StrlNode& node)
{
	os << "[Esterel Node] - [" << node.node_id << "] [Dispatched=" << (node.dispatched ? "TRUE" : "FALSE") << "] [COMPLETED=" << (node.completed ? "TRUE" : "FALSE") << "]; Addr: " << &node << std::endl;
	os << "Input:" << std::endl;
	for (unsigned int i = 0; i < node.input.size(); ++i)
	{
		//os << "\t" << node.input[i]->edge_name << " - " << (node.input[i]->active ? "ACTIVATED" : "INACTIVE") << std::endl;
		os << *node.input[i] << std::endl;
	}
	os << "Output:" << std::endl;
	for (unsigned int i = 0; i < node.output.size(); ++i)
	{
		//os << "\t" << node.output[i]->edge_name << " - " << (node.output[i]->active ? "ACTIVATED" : "INACTIVE") << std::endl;
		os << *node.output[i] << std::endl;
	}
	os << "[\Esterel Node]" << std::endl;
	return os;
};
