#include "rosplan_knowledge_base/MongoInterface.h"

namespace KCL_rosplan {
    MongoInterface::MongoInterface(std::string dbHost, std::string dbPort, std::string dbName)
        : dbName(dbName)
    {
        ns_k = dbName+".knowledge";
        ns_g = dbName+".goals";
        mongo::client::initialize();
        try {
            ROS_INFO_STREAM("Connecting to DB: " << dbHost << ":" << dbPort);
            client.connect(dbHost+":"+dbPort);
            ROS_INFO("connected.");
        } catch( const mongo::DBException &e ) {
            ROS_ERROR_STREAM("Connection error: Caught " << e.what());
        }
    }
    
    // Helper functions
    mongo::BSONObj MongoInterface::knowledgeItemToBson(rosplan_knowledge_msgs::KnowledgeItem &ki) {
        mongo::BSONArrayBuilder ab;
        for(int vit = 0; vit < ki.values.size(); vit++) {
            ab.append(BSON("key" << ki.values[vit].key << "value" << ki.values[vit].value));
        }
        mongo::BSONObj p = BSON(
                    "instance_type" << ki.instance_type 
                    << "instance_name" << ki.instance_name
                    << "attribute_name" << ki.attribute_name
                    << "function_value" << ki.function_value
                    << "is_negative" << ki.is_negative
                    << "knowledge_type" << ki.knowledge_type
                    << "values" << ab.arr()
                    );
        return p;
    }
    
    rosplan_knowledge_msgs::KnowledgeItem MongoInterface::bsonToKnowledgeItem(mongo::BSONObj b) {
        rosplan_knowledge_msgs::KnowledgeItem ki;
        ki.instance_type = b.getField("instance_type").String();
        ki.instance_name = b.getField("instance_name").String();
        ki.attribute_name = b.getField("attribute_name").String();
        ki.function_value = b.getField("function_value").Double();
        ki.is_negative = (bool)b.getField("is_negative").Int();
        ki.knowledge_type = b.getField("knowledge_type").Int();
        std::vector<mongo::BSONElement> a;
        b.getObjectField("values").elems(a);
        for(int i = 0; i < a.size(); i++) {
            diagnostic_msgs::KeyValue kv;
            kv.key = a[i].embeddedObject().getField("key").String();
            kv.value = a[i].embeddedObject().getField("value").String();
            ki.values.push_back(kv);
        }
        return ki;
    }
    
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getEntries(std::string ns, mongo::Query query) {
        db_cursor cursor = client.query(ns, query);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> ret;
        while(cursor->more()) {
            ret.push_back(bsonToKnowledgeItem(cursor->next()));
        }
        return ret;
    }
    
    void MongoInterface::addEntry(std::string ns, mongo::BSONObj b) {
        client.insert(ns, b);
        client.ensureIndex(ns, BSON("knowledge_type" << 1 << "instance_type" << 1 << "instance_name" << 1 << "attribute_name" << 1));
    }

    bool MongoInterface::isEntry(std::string ns, mongo::BSONObj b) {
        db_cursor cursor = client.query(ns, b);
        return cursor->more();
    }
    
    void MongoInterface::removeEntry(std::string ns, mongo::BSONObj b) {
        client.remove(ns, b);
    }

    mongo::BSONObj MongoInterface::findMongoEntry(std::string ns, mongo::Query query) {
         mongo::BSONObj o = client.findOne(ns, query);
         if(o.isEmpty())
             throw MongoException();
         return o;
    }
    
    mongo::BSONObj MongoInterface::generateFFQuery(rosplan_knowledge_msgs::KnowledgeItem &ki){
        mongo::BSONArrayBuilder ab;
        for(int vit = 0; vit < ki.values.size(); vit++) {
            ab.append(BSON("key" << ki.values[vit].key << "value" << ki.values[vit].value));
        }
        mongo::BSONObjBuilder b;
        b.append("attribute_name", ki.attribute_name);
        b.append("is_negative", ki.is_negative);
        b.append("values", ab.arr());
        return b.obj();
    }

    mongo::BSONObj MongoInterface::generateFFQuery(rosplan_knowledge_msgs::KnowledgeItem &ki, int type1) {
        return (mongo::BSONObjBuilder().appendElements(generateFFQuery(ki))<<"knowledge_type"<<type1).obj();
    }

    mongo::BSONObj MongoInterface::generateFFQuery(rosplan_knowledge_msgs::KnowledgeItem &ki, int type1, int type2) {
        return (mongo::BSONObjBuilder().appendElements(generateFFQuery(ki))<<"$or"<<BSON_ARRAY(BSON("knowledge_type"<<type1)<<BSON("knowledge_type"<<type2))).obj();
    }
    
    // Knowledge specific functions
    
    // Instances
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getInstances(std::string instance_type, std::string instance_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::INSTANCE);
        if(instance_type.compare("")!=0) b.append("instance_type", instance_type);
        if(instance_name.compare("")!=0) b.append("instance_name", instance_name);
        return getKnowledge(b.obj());
    }
    
    void MongoInterface::removeInstances(std::string instance_type, std::string instance_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::INSTANCE);
        if(instance_type.compare("")!=0) b.append("instance_type", instance_type);
        if(instance_name.compare("")!=0) b.append("instance_name", instance_name);
        return removeKnowledge(b.obj());
    }

    // Facts
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getFacts(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FACT);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return getKnowledge(b.obj());
    }
    
    void MongoInterface::removeFacts(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FACT);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return removeKnowledge(b.obj());
    }

    bool MongoInterface::isFact(rosplan_knowledge_msgs::KnowledgeItem &ki) {
        if(ki.attribute_name.compare("")==0) return false;
        return isFact(generateFFQuery(ki, rosplan_knowledge_msgs::KnowledgeItem::FACT));
    }

    void MongoInterface::removeFactsAndFunctions(rosplan_knowledge_msgs::KnowledgeItem &ki) {
        if(ki.attribute_name.compare("")==0) return;
        findAndRemoveKnowledge(generateFFQuery(ki, rosplan_knowledge_msgs::KnowledgeItem::FACT, rosplan_knowledge_msgs::KnowledgeItem::FUNCTION));
    }
    
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getContainsInstanceFacts(std::string instance_name) {
        return getKnowledge(BSON(
            "knowledge_type" << rosplan_knowledge_msgs::KnowledgeItem::FACT <<
            "$or" << BSON_ARRAY(
                BSON("instance_name" << instance_name) <<
                BSON("values" << BSON("$elemMatch" << BSON("value" << instance_name)))
            )
        ));
    }

    // Functions
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getFunctions(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FUNCTION);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return getKnowledge(b.obj());
    }
    
    void MongoInterface::removeFunctions(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FUNCTION);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return removeKnowledge(b.obj());
    }

    bool MongoInterface::isFunction(rosplan_knowledge_msgs::KnowledgeItem &ki) {
        if(ki.attribute_name.compare("")==0) return false;
        return isFunction(generateFFQuery(ki, rosplan_knowledge_msgs::KnowledgeItem::FUNCTION));
    }
    
    // Goals
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getGoals(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FACT);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return getGoals(b.obj());
    }
    
    void MongoInterface::removeGoals(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FACT);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return removeEntry(ns_g, b.obj());
    }

    void MongoInterface::removeGoal(rosplan_knowledge_msgs::KnowledgeItem &ki) {
        return removeEntry(ns_g, ki);
    }

    void MongoInterface::findAndRemoveGoal(rosplan_knowledge_msgs::KnowledgeItem &ki) {
        if(ki.attribute_name.compare("")==0) return;
        removeEntry(ns_g, findMongoEntry(ns_g, generateFFQuery(ki, rosplan_knowledge_msgs::KnowledgeItem::FACT, rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)));
    }
    
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getContainsInstanceGoals(std::string instance_name) {
        return getGoals(BSON(
            "knowledge_type" << rosplan_knowledge_msgs::KnowledgeItem::FACT <<
            "$or" << BSON_ARRAY(
                BSON("instance_name" << instance_name) <<
                BSON("values" << BSON("$elemMatch" << BSON("value" << instance_name)))
            )
        ));
    }
}
