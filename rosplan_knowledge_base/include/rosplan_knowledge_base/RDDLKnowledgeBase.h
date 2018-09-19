#ifndef KCL_knowledgebase
#define KCL_knowledgebase

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

#include "KnowledgeComparitor.h"
#include "RDDLTaskParser.h"

#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"
#include "VALVisitorProblem.h"


namespace KCL_rosplan {

	class RDDLKnowledgeBase
	{
	private:

		/* adding items to the knowledge base */
		// void addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg); // FIXME handled by syperclass?
		//void addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg); // FIXME handled by syperclass?
		//void addMissionMetric(rosplan_knowledge_msgs::KnowledgeItem &msg); // FIXME handled by syperclass?

		/* removing items from the knowledge base */
		//void removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg); // FIXME handled by syperclass?
		//void removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg); // FIXME handled by syperclass?
		//void removeMissionMetric(rosplan_knowledge_msgs::KnowledgeItem &msg); // FIXME handled by syperclass?

		/* Converts RDDL params to ROS-like params */
		std::vector<diagnostic_msgs::KeyValue> getTypedParams(const std::vector<Parameter*>& params);

		/* Get object instances */
        std::map<std::string, std::vector<std::string> > getInstances(const std::map<std::string, Object*>& instance_objects);

	public:

		/* parsing domain using VAL */
		RDDLTaskParser domain_parser;


		/* PDDL model (current state) */
		std::map<std::string, std::vector<std::string> > model_constants;
		std::map<std::string, std::vector<std::string> > model_instances;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_facts;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_functions;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_goals;
        rosplan_knowledge_msgs::KnowledgeItem model_metric;

		/* timed initial literals */
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_timed_initial_literals;

		/* conditional planning */
		std::vector<std::vector<rosplan_knowledge_msgs::KnowledgeItem> > model_oneof_constraints;
		bool use_unknowns;

        /* add the initial state to the knowledge base */
        void addInitialState(); // FIXME change in superclass?

		/* service methods for querying the model */
		bool queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res);

		/* service methods for fetching the current state */
		// bool getInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res); // FIXME handled by syperclass?
		// bool getPropositions(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res); // FIXME handled by syperclass?
		// bool getFunctions(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res); // FIXME handled by syperclass?
		// bool getGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res); // FIXME handled by syperclass?
		// bool getMetric(rosplan_knowledge_msgs::GetMetricService::Request  &req, rosplan_knowledge_msgs::GetMetricService::Response &res); // FIXME handled by syperclass?
		// bool getTimedKnowledge(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res); // FIXME handled by syperclass?

		/* service methods for adding and removing items to and from the current state */
		// bool updateKnowledgeArray(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res); // FIXME handled by syperclass?
		// bool updateKnowledge(rosplan_knowledge_msgs::KnowledgeUpdateService::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateService::Response &res); // FIXME handled by syperclass?
		// bool clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res); // FIXME handled by syperclass?

		/* service methods for fetching the domain details */
		bool getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request  &req, rosplan_knowledge_msgs::GetDomainNameService::Response &res);
		bool getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res);
		bool getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res);
		bool getFunctionPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res);
		bool getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res); //TODO
		bool getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res);//TODO
		bool getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res);

		/* service methods for conditional planning */
		// bool updateKnowledgeConstraintsOneOf(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res); // FIXME handled by syperclass?
		// TODO bool getCurrentConstraintsOneOf(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);

		/* main loop */
		//void runKnowledgeBase(); // FIXME handled by superclass?
	};
}
#endif
