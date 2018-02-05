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
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"

#include "KnowledgeComparitor.h"
#include "PlanFilter.h"
#include "DomainParser.h"
#include "InitialStateParser.h"
#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"
#include "VALVisitorProblem.h"

#ifndef KCL_knowledgebase
#define KCL_knowledgebase

namespace KCL_rosplan {

    class KnowledgeBase
    {
    private:

        // adding and removing items to and from the knowledge base
        void addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
        void addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);
        void removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
        void removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);

    public:

        // conditional planning flags
        bool use_unknowns;

        // domain
        DomainParser domain_parser;

        // initial state from problem file
        InitialStateParser initialState_parser;

        // model
        // TODO add constants to KB
        std::map<std::string, std::vector<std::string> > model_constants;
        std::map<std::string, std::vector<std::string> > model_instances;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_facts;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_functions;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_goals;

        // conditional planning constraints
        std::vector<std::vector<rosplan_knowledge_msgs::KnowledgeItem> > model_oneof_constraints;

        /* fetching the domain */
        bool getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request  &req, rosplan_knowledge_msgs::GetDomainNameService::Response &res);
        bool getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res);
        bool getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res);
        bool getFunctions(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res);
        bool getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res);
        bool getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res);
        bool getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res);

        // checking the model
        // TODO function inequalities in queryKnowledge
        bool queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res);

        // fetching the model
        bool getCurrentInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res);
        bool getCurrentKnowledge(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
        bool getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
        // TODO bool getCurrentConstraintsOneOf(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);


        // add the initial state to the knowledge base
        void addInitialState(VAL::domain* domain, VAL::problem* problem);


        // adding and removing items to and from the knowledge base
        bool updateKnowledge(rosplan_knowledge_msgs::KnowledgeUpdateService::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateService::Response &res);
        bool updateKnowledgeArray(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res);
        bool clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

        // conditional planning services
        bool updateKnowledgeConstraintsOneOf(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res);
        // TODO bool updateKnowledgeConstraintsUnknowns(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res);
    };
}
#endif