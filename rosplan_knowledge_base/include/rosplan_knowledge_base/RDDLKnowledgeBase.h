#ifndef KCL_RDDL_Knowledgebase
#define KCL_RDDL_Knowledgebase

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
#include "RDDLTaskParser.h"

#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"
#include "VALVisitorProblem.h"


namespace KCL_rosplan {

	class RDDLKnowledgeBase : public KnowledgeBase
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

        /* Get facts and functions from the task */
        void loadFactsAndFunctions();

		/* Get the metric from the task */
        void loadMetric();

        /* Just fils the header for a fact */
        rosplan_knowledge_msgs::KnowledgeItem fillKI(const ParametrizedVariable* var, const std::vector<Parameter *> &params, double initialValue);
        /* Adds all the functions and facts that have a default value in the initial state with their grounded parameters */
        void fillKIAddAllGroundedParameters(const ParametrizedVariable *var,
											std::map<std::string, rosplan_knowledge_msgs::KnowledgeItem> &factsfuncs,
											rosplan_knowledge_msgs::KnowledgeItem &item,
											std::string ground_params = "", int param_index = 0);


	public:

		/* parsing domain using VAL */
		RDDLTaskParser domain_parser;

		/* parse domain and probelm files */
		inline void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path);

        /* add the initial state to the knowledge base */
        void addInitialState(); // FIXME change in superclass?

		/* service methods for querying the model */
		// bool queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res); // FIXME handled by syperclass?

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
