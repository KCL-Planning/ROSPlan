#ifndef KCL_PDDL_Knowledgebase
#define KCL_PDDL_Knowledgebase

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "std_srvs/Empty.h"

#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"

#include "rosplan_knowledge_msgs/GetDomainNameService.h"
#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"

#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetMetricService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#include "KnowledgeBase.h"
#include "KnowledgeComparitor.h"
#include "PDDLDomainParser.h"
#include "PDDLProblemParser.h"
#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"
#include "VALVisitorProblem.h"


namespace KCL_rosplan {

	class PDDLKnowledgeBase : public KnowledgeBase {
	private:
		/* parsing domain using VAL */
		PDDLDomainParser domain_parser;

		/* initial state from problem file using VAL */
		PDDLProblemParser problem_parser;
	public:
		PDDLKnowledgeBase(ros::NodeHandle& n) : KnowledgeBase(n) {};
		~PDDLKnowledgeBase() = default;

		/* parse domain and probelm files */
		void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) override;

        /* add the initial state to the knowledge base */
        void addInitialState() override;
        void addConstants() override;

		/* service methods for fetching the domain details */
		bool getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request  &req, rosplan_knowledge_msgs::GetDomainNameService::Response &res) override;
		bool getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res) override;
		bool getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) override;
		bool getFunctionPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) override;
		bool getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res) override;
		bool getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) override;
		bool getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) override;
	};
}
#endif
