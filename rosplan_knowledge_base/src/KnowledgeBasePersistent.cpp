#include "rosplan_knowledge_base/KnowledgeBasePersistent.h"

namespace KCL_rosplan {

    KnowledgeBasePersistent::KnowledgeBasePersistent(std::string dbHost, std::string dbPort, std::string dbName) {
        mongo_interface = new MongoInterface(dbHost, dbPort, dbName);
    }
    
    KnowledgeBasePersistent::~KnowledgeBasePersistent() {
        delete mongo_interface;
    }

    /*-----------------*/
    /* knowledge query */
    /*-----------------*/

    bool KnowledgeBasePersistent::queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res) {

        res.all_true = true;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
        for(iit = req.knowledge.begin(); iit!=req.knowledge.end(); iit++) {

            bool present = false;
            if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {

                // check if instance exists
                try {
                    present = mongo_interface->isInstance(*iit);
                } catch (std::exception &e) {
                    ROS_WARN_STREAM(e.what());
                }

            } else if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION) {

                // check if function exists; TODO inequalities
                try {
                    present = mongo_interface->isFunction(*iit);
                } catch (std::exception &e) {
                    ROS_WARN_STREAM(e.what());
                }

            } else if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

                // check if fact is true
                try {
                    present = mongo_interface->isFact(*iit);
                } catch (std::exception &e) {
                    ROS_WARN_STREAM(e.what());
                }

            }

            if(!present) {
                res.all_true = false;
                res.results.push_back(false);
                res.false_knowledge.push_back(*iit);
            } else {
                res.results.push_back(true);
            }
        }

        return true;
    }

    /*----------------*/
    /* removing items */
    /*----------------*/

    void KnowledgeBasePersistent::removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg) {

        if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {
            ROS_INFO("KCL: (KB) Removing instances (%s, %s)", msg.instance_type.c_str(), msg.instance_name.c_str());
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> to_delete = mongo_interface->getContainsInstanceFacts(msg.instance_name);
            std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
            for(iit = to_delete.begin(); iit != to_delete.end(); ++iit) {
                ROS_INFO("KCL: (KB) Removing domain attribute (%s)", iit->attribute_name.c_str());
                plan_filter.checkFilters(*iit, false);
                try {
                    mongo_interface->removeKnowledge(*iit);
                } catch (std::exception &e) {
                    ROS_WARN_STREAM(e.what());
                }
            }
            to_delete = mongo_interface->getContainsInstanceGoals(msg.instance_name);
            for(iit = to_delete.begin(); iit != to_delete.end(); ++iit) {
                ROS_INFO("KCL: (KB) Removing goal (%s)", iit->attribute_name.c_str());
                plan_filter.checkFilters(*iit, false);
                try {
                    mongo_interface->removeGoal(*iit);
                } catch (std::exception &e) {
                    ROS_WARN_STREAM(e.what());
                }
            }
            // Remove instances
            try {
                mongo_interface->removeInstances(msg.instance_type, msg.instance_name);
            } catch (std::exception &e) {
                ROS_WARN_STREAM(e.what());
            }

        } else {

            // remove domain attribute (function/fact) from knowledge base
            ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
            plan_filter.checkFilters(msg, false);
            try {
                mongo_interface->removeFactsAndFunctions(msg);
            } catch (std::exception &e) {
                ROS_WARN_STREAM(e.what());
            }

        }
    }

    /**
     * remove everything
     */
    bool KnowledgeBasePersistent::clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

        ROS_INFO("KCL: (KB) Removing whole model");

        // model
        mongo_interface->removeKnowledge(mongo::BSONObj());
        mongo_interface->removeGoals();
    }

    /**
     * remove all goals
     */
    bool KnowledgeBasePersistent::clearGoals(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

        ROS_INFO("KCL: (KB) Removing all goals");

        mongo_interface->removeGoals();
    }

    /**
     * remove mission goal
     */
    void KnowledgeBasePersistent::removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {

        bool changed = false;

        mongo_interface->findAndRemoveGoal(msg);

        if(changed) {            
            rosplan_knowledge_msgs::Notification notMsg;
            notMsg.function = rosplan_knowledge_msgs::Notification::REMOVED;
            notMsg.knowledge_item = msg;
            plan_filter.notification_publisher.publish(notMsg);
        }
    }

    /*--------------*/
    /* adding items */
    /*--------------*/

    /*
     * add an instance, domain predicate, or function to the knowledge base
     */
    void KnowledgeBasePersistent::addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg) {
        if(!mongo_interface->isKnowledge(msg)) {
            ROS_INFO("KCL: (KB) Adding knowledge");
            mongo_interface->addKnowledge(msg);
            plan_filter.checkFilters(msg, true);
        } else {
            ROS_INFO("KCL: (KB) Knowledge already in KB");
        }
    }

    /*
     * add mission goal to knowledge base
     */
    void KnowledgeBasePersistent::addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {
        if(!mongo_interface->isGoal(msg)) {
            ROS_INFO("KCL: (KB) Adding goal");
            mongo_interface->addGoal(msg);
        } else {
            ROS_INFO("KCL: (KB) Goal already in KB");
        }
    }

    /*----------------*/
    /* fetching items */
    /*----------------*/

    bool KnowledgeBasePersistent::getCurrentInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res) {
    
        // fetch the instances of the correct type
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> i = mongo_interface->getInstances(req.type_name);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
        for(iit=i.begin(); iit != i.end(); iit++) {
            res.instances.push_back((*iit).instance_name);
        }
        return true;
    }

    bool KnowledgeBasePersistent::getCurrentKnowledge(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {
        mongo::BSONObjBuilder b;
        b.append("$or", BSON_ARRAY(
                     BSON("knowledge_type" << rosplan_knowledge_msgs::KnowledgeItem::FACT) << 
                     BSON("knowledge_type" << rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
                 ));
        if(req.predicate_name.compare("")!=0) b.append("attribute_name", req.predicate_name);

        std::vector<rosplan_knowledge_msgs::KnowledgeItem> i = mongo_interface->getKnowledge(b.obj());
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
        for(iit=i.begin(); iit != i.end(); iit++) {
            res.attributes.push_back(*iit);
        }
        return true;
    }

    bool KnowledgeBasePersistent::getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {

        std::vector<rosplan_knowledge_msgs::KnowledgeItem> i = mongo_interface->getGoals(req.predicate_name);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
        for(iit=i.begin(); iit != i.end(); iit++) {
            res.attributes.push_back(*iit);
        }
        return true;
    }

} // close namespace
