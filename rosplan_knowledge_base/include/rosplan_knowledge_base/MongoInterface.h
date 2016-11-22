#include <iostream>
#include <vector>
#include <exception>

#include <ros/ros.h>
#include<rosplan_knowledge_msgs/KnowledgeItem.h>

#include <mongo/client/dbclient.h>
#include <mongo/bson/bson.h>

#ifndef MONGOINTERFACE_H
#define MONGOINTERFACE_H


namespace KCL_rosplan {

class MongoException : public std::exception
{
    virtual const char* what() const throw() {
        return "Database query failed: No such entry.";
    }
};

typedef std::auto_ptr<mongo::DBClientCursor> db_cursor;

class MongoInterface
    {
    private:
        std::string ns_k, ns_g;

        mongo::BSONObj generateFFQuery(rosplan_knowledge_msgs::KnowledgeItem &ki);
    public:
        // DB params and client
        std::string dbName;
        mongo::DBClientConnection client;

        MongoInterface(std::string dbHost, std::string dbPort, std::string dbName);

        //helper functions
        mongo::BSONObj knowledgeItemToBson(rosplan_knowledge_msgs::KnowledgeItem &ki);
        rosplan_knowledge_msgs::KnowledgeItem bsonToKnowledgeItem(mongo::BSONObj b);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getEntries(std::string ns, mongo::Query query);
        void addEntry(std::string ns, rosplan_knowledge_msgs::KnowledgeItem &ki) { return addEntry(ns, knowledgeItemToBson(ki)); }
        void addEntry(std::string ns, mongo::BSONObj b);
        bool isEntry(std::string ns, rosplan_knowledge_msgs::KnowledgeItem &ki) { return isEntry(ns, knowledgeItemToBson(ki)); }
        bool isEntry(std::string ns, mongo::BSONObj b);
        void removeEntry(std::string ns, rosplan_knowledge_msgs::KnowledgeItem &ki) { return removeEntry(ns, knowledgeItemToBson(ki)); }
        void removeEntry(std::string ns, mongo::BSONObj b);
        rosplan_knowledge_msgs::KnowledgeItem findKIEntry(std::string ns, mongo::Query query) { return bsonToKnowledgeItem(findMongoEntry(ns, query)); }
        mongo::BSONObj findMongoEntry(std::string ns, mongo::Query query);
        mongo::BSONObj generateFFQuery(rosplan_knowledge_msgs::KnowledgeItem &ki, int type1);
        mongo::BSONObj generateFFQuery(rosplan_knowledge_msgs::KnowledgeItem &ki, int type1, int type2);

        // Convenience functions
        void addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &ki) { return addKnowledge(knowledgeItemToBson(ki)); }
        void addKnowledge(mongo::BSONObj b) { return addEntry(ns_k, b); }
        void removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &ki) { return removeKnowledge(knowledgeItemToBson(ki)); }
        void removeKnowledge(mongo::BSONObj b) { return removeEntry(ns_k, b); }
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getKnowledge(mongo::Query query) { return getEntries(ns_k, query); }
        rosplan_knowledge_msgs::KnowledgeItem findKnowledge(mongo::Query query) { return findKIEntry(ns_k, query); }
        void findAndRemoveKnowledge(mongo::Query query) { removeEntry(ns_k, findMongoEntry(ns_k, query)); }
        void addGoal(rosplan_knowledge_msgs::KnowledgeItem &ki) { return addGoal(knowledgeItemToBson(ki)); }
        void addGoal(mongo::BSONObj b) { return addEntry(ns_g, b); }
        bool isKnowledge(rosplan_knowledge_msgs::KnowledgeItem &ki) { return isKnowledge(knowledgeItemToBson(ki)); }
        bool isKnowledge(mongo::BSONObj b) { return isEntry(ns_k, b); }

        // Knowledge specific functions

        // Instances
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getInstances(std::string instance_type="", std::string instance_name="");
        void removeInstances(std::string instance_type="", std::string instance_name="");
        bool isInstance(rosplan_knowledge_msgs::KnowledgeItem &ki) { return isInstance(knowledgeItemToBson(ki)); }
        bool isInstance(mongo::BSONObj b) { return isKnowledge(b); }
        
        // Facts
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getFacts(std::string attribute_name="");
        void removeFacts(std::string attribute_name="");
        bool isFact(rosplan_knowledge_msgs::KnowledgeItem &ki);
        bool isFact(mongo::BSONObj b) { return isKnowledge(b); }
        void removeFactsAndFunctions(rosplan_knowledge_msgs::KnowledgeItem &ki);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getContainsInstanceFacts(std::string instance_name);

        // Functions
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getFunctions(std::string attribute_name="");
        void removeFunctions(std::string attribute_name="");
        bool isFunction(rosplan_knowledge_msgs::KnowledgeItem &ki);
        bool isFunction(mongo::BSONObj b) { return isKnowledge(b); }

        // Goals
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getGoals(std::string attribute_name="");
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getGoals(mongo::Query query) { return getEntries(ns_g, query); }
        void removeGoals(std::string attribute_name="");
        void removeGoal(rosplan_knowledge_msgs::KnowledgeItem &ki);
        bool isGoal(rosplan_knowledge_msgs::KnowledgeItem &ki) { return isGoal(knowledgeItemToBson(ki)); }
        bool isGoal(mongo::BSONObj b) { return isEntry(ns_g, b); }
        rosplan_knowledge_msgs::KnowledgeItem findGoal(mongo::Query query) { return findKIEntry(ns_g, query); }
        void findAndRemoveGoal(rosplan_knowledge_msgs::KnowledgeItem &ki);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getContainsInstanceGoals(std::string instance_name);
    };

}

#endif // MONGOINTERFACE_H
