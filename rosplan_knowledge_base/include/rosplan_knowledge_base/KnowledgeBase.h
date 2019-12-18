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
#include "rosplan_knowledge_msgs/SetNamedBool.h"

#include "rosplan_knowledge_msgs/GetDomainNameService.h"
#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "rosplan_knowledge_msgs/StatusUpdate.h"

#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetMetricService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#include "KnowledgeComparitor.h"


namespace KCL_rosplan {

	class KnowledgeBase
	{
	private:
        ros::ServiceServer domainServer1; // getDomainName
        ros::ServiceServer domainServer2; // getTypes
        ros::ServiceServer domainServer3; // getPredicates
        ros::ServiceServer domainServer4; // getFunctionPredicates
        ros::ServiceServer domainServer5; // getOperators
        ros::ServiceServer domainServer6; // getOperatorDetails
        ros::ServiceServer domainServer7; // getPredicateDetails

        // query knowledge
        ros::ServiceServer queryServer; // queryKnowledge
        ros::ServiceServer senseServer; // queryKnowledge

        // update knowledge
        ros::ServiceServer updateServer0; // clearKnowledge
        ros::ServiceServer updateServer1; // updateKnowledge
        ros::ServiceServer updateServer2; // updateKnowledgeArray
        ros::ServiceServer updateServer3; // updateKnowledgeConstraintsOneOf

        // fetch knowledge
        ros::ServiceServer stateServer1; // getInstances
        ros::ServiceServer stateServer2; // getPropositions
        ros::ServiceServer stateServer3; // getFunctions
        ros::ServiceServer stateServer4; // getTimedKnowledge
        ros::ServiceServer stateServer5; // getGoals
        ros::ServiceServer stateServer6; // getMetric

	protected:
    
        ros::Publisher status_pub;

		/* adding items to the knowledge base */
		void addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
		void addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);
		void addMissionMetric(rosplan_knowledge_msgs::KnowledgeItem &msg);

		/* removing items from the knowledge base */
		void removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
        virtual void removeFact(const rosplan_knowledge_msgs::KnowledgeItem &msg);

        void removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);
		void removeMissionMetric(rosplan_knowledge_msgs::KnowledgeItem &msg);

		/* PDDL model (persistent state) */
		std::map<std::string, std::vector<std::string> > domain_constants;

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

		/* sensing information */
		std::map<std::string, bool> sensed_predicates;

        ros::NodeHandle _nh;

        double _kb_rate;
    public:

		KnowledgeBase(ros::NodeHandle& n);
		~KnowledgeBase() = default;

        bool use_unknowns;

		/* parse domain and probelm files */
		virtual void parseDomain(const std::string& domain_file_path, const std::string& problem_file_path) =0;

        /* add the initial state to the knowledge base */
        virtual void addInitialState() =0;
    	virtual void addConstants() =0;

		/* service methods for querying the model */
		bool queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res);

		/* service methods for fetching the current state */
		bool getInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res);
		bool getPropositions(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
		bool getFunctions(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
		bool getGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
		bool getMetric(rosplan_knowledge_msgs::GetMetricService::Request  &req, rosplan_knowledge_msgs::GetMetricService::Response &res);
		bool getTimedKnowledge(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);

		/* service methods for adding and removing items to and from the current state */
        bool updateKnowledgeArray(ros::ServiceEvent<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response> &event);
		//bool updateKnowledgeArray(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res);
		bool updateKnowledge(rosplan_knowledge_msgs::KnowledgeUpdateService::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateService::Response &res);
		bool clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

		/* service methods for fetching the domain details */
		virtual bool getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request  &req, rosplan_knowledge_msgs::GetDomainNameService::Response &res) =0;
		virtual bool getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res) =0;
		virtual bool getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) =0;
		virtual bool getFunctionPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) =0;
		virtual bool getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res) =0;
		virtual bool getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) =0;
		virtual bool getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) =0;

		/* service methods for conditional planning */
		bool updateKnowledgeConstraintsOneOf(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res);
		// TODO bool getCurrentConstraintsOneOf(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);

		/* service methods for sensed predicates */
		bool setSensedPredicate(rosplan_knowledge_msgs::SetNamedBool::Request  &req, rosplan_knowledge_msgs::SetNamedBool::Response &res);

        /* publish status */
        void publishStatusUpdate(ros::Time &time, std::string &caller_id);

		/* main loop */
		void runKnowledgeBase();
	};
}
#endif
