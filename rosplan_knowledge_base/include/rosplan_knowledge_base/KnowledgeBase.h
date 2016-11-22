#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "std_srvs/Empty.h"

#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"

#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"

#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"

#include "KnowledgeComparitor.h"
#include "PlanFilter.h"
#include "DomainParser.h"
#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"

#ifndef KCL_knowledgebase
#define KCL_knowledgebase

namespace KCL_rosplan {

	class KnowledgeBase
	{
	protected:

		// visit controllers for ROS message packing
		VALVisitorOperator op_visitor;
		VALVisitorPredicate pred_visitor;

		// adding and removing items to and from the knowledge base
		virtual void addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
		virtual void addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);
		virtual void removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
		virtual void removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);

	public:

		// domain
		DomainParser domain_parser;

		// model
		std::map<std::string, std::vector<std::string> > model_instances;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_facts;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_functions;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_goals;

		// plan and mission filter
		PlanFilter plan_filter;

		/* fetching the domain */
		virtual bool getTyes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res);		
		virtual bool getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res);
		virtual bool getFunctions(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res);
		virtual bool getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res);
		virtual bool getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res);
		virtual bool getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res);

		// checking the model
		virtual bool queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res);

		// fetching the model
		virtual bool getCurrentInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res);
		virtual bool getCurrentKnowledge(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
		virtual bool getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);

		// adding and removing items to and from the knowledge base
		virtual bool updateKnowledge(rosplan_knowledge_msgs::KnowledgeUpdateService::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateService::Response &res);
		virtual bool updateKnowledgeArray(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res);
        virtual bool clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
        virtual bool clearGoals(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
	};
}
#endif
