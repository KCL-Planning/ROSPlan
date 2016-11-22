#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include <std_srvs/Empty.h>

#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>

#include <rosplan_knowledge_msgs/GetDomainTypeService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorService.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <rosplan_knowledge_msgs/DomainFormula.h>

#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>

#include <rosplan_knowledge_msgs/Notification.h>
#include <rosplan_knowledge_msgs/Filter.h>

#include "KnowledgeBase.h"
#include "KnowledgeComparitor.h"
#include "PlanFilter.h"
#include "DomainParser.h"
#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"
#include "MongoInterface.h"

#ifndef KCL_knowledgebasepersistent
#define KCL_knowledgebasepersistent

namespace KCL_rosplan {

    class KnowledgeBasePersistent : public KnowledgeBase
    {
    protected:
        MongoInterface *mongo_interface;
        
        // adding and removing items to and from the knowledge base
        void addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
        void addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);
        void removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
        void removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);

    public:
        KnowledgeBasePersistent(std::string dbHost, std::string dbPort, std::string dbName);
        ~KnowledgeBasePersistent();

        // checking the model
        bool queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res);

        // fetching the model
        bool getCurrentInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res);
        bool getCurrentKnowledge(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
        bool getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);

        // adding and removing items to and from the knowledge base
        bool clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
        bool clearGoals(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

    };
}
#endif
