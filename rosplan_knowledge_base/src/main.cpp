#include <ros/ros.h>

#include "rosplan_knowledge_base/KnowledgeBase.h"

#ifdef USE_MONGO_CXX
	#include "rosplan_knowledge_base/KnowledgeBasePersistent.h"
#endif

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_knowledge_base");
	ros::NodeHandle n;

	// parameters
	std::string domainPath;
	n.param("/rosplan/domain_path", domainPath, std::string("common/domain.pddl"));
	std::string dbHost = "localhost";
	n.param("/rosplan/db_host", dbHost, dbHost);
	std::string dbPort = "62345";
	n.param("/rosplan/db_port", dbPort, dbPort);
	std::string dbName = "kcl_knowledge_base";
	n.param("/rosplan/knowledge_base/db_name", dbName, dbName);
	bool persistent = false;
	n.param("/rosplan/knowledge_base/persistent", persistent, persistent);

	KCL_rosplan::KnowledgeBase *kb;
	#ifdef USE_MONGO_CXX
		if(!persistent) {
			kb = new KCL_rosplan::KnowledgeBase();
		} else {
			kb = new KCL_rosplan::KnowledgeBasePersistent(dbHost, dbPort, dbName);
		}
	#else
		kb = new KCL_rosplan::KnowledgeBase();
		if(persistent)
			ROS_WARN("Knowledge base has been compiled without mongodb support. No persistency enabled.");
	#endif
	ROS_INFO("KCL: (KB) Parsing domain");
	kb->domain_parser.domain_parsed = false;
	kb->domain_parser.parseDomain(domainPath);

	// fetch domain info
	ros::ServiceServer typeServer = n.advertiseService("/kcl_rosplan/get_domain_types", &KCL_rosplan::KnowledgeBase::getTyes, kb);
	ros::ServiceServer predicateServer = n.advertiseService("/kcl_rosplan/get_domain_predicates", &KCL_rosplan::KnowledgeBase::getPredicates, kb);
	ros::ServiceServer functionServer = n.advertiseService("/kcl_rosplan/get_domain_functions", &KCL_rosplan::KnowledgeBase::getFunctions, kb);
	ros::ServiceServer operatorServer = n.advertiseService("/kcl_rosplan/get_domain_operators", &KCL_rosplan::KnowledgeBase::getOperators, kb);

	ros::ServiceServer opDetailsServer = n.advertiseService("/kcl_rosplan/get_domain_operator_details", &KCL_rosplan::KnowledgeBase::getOperatorDetails, kb);
	ros::ServiceServer predDetailsServer = n.advertiseService("/kcl_rosplan/get_domain_predicate_details", &KCL_rosplan::KnowledgeBase::getPredicateDetails, kb);

	// query knowledge
	ros::ServiceServer queryServer = n.advertiseService("/kcl_rosplan/query_knowledge_base", &KCL_rosplan::KnowledgeBase::queryKnowledge, kb);

	// update knowledge
	ros::ServiceServer updateServer1 = n.advertiseService("/kcl_rosplan/update_knowledge_base", &KCL_rosplan::KnowledgeBase ::updateKnowledge, kb);
	ros::ServiceServer updateServer2 = n.advertiseService("/kcl_rosplan/update_knowledge_base_array", &KCL_rosplan::KnowledgeBase::updateKnowledgeArray, kb);
	ros::ServiceServer clearServer = n.advertiseService("/kcl_rosplan/clear_knowledge_base", &KCL_rosplan::KnowledgeBase::clearKnowledge, kb);
    ros::ServiceServer clearGoalsServer = n.advertiseService("/kcl_rosplan/clear_goals", &KCL_rosplan::KnowledgeBase::clearGoals, kb);

	// fetch knowledge
	ros::ServiceServer currentInstanceServer = n.advertiseService("/kcl_rosplan/get_current_instances", &KCL_rosplan::KnowledgeBase::getCurrentInstances, kb);
	ros::ServiceServer currentKnowledgeServer = n.advertiseService("/kcl_rosplan/get_current_knowledge", &KCL_rosplan::KnowledgeBase::getCurrentKnowledge, kb);
	ros::ServiceServer currentGoalServer = n.advertiseService("/kcl_rosplan/get_current_goals", &KCL_rosplan::KnowledgeBase::getCurrentGoals, kb);

	// planning and mission filter
	kb->plan_filter.notification_publisher = n.advertise<rosplan_knowledge_msgs::Notification>("/kcl_rosplan/notification", 10, true);
	ros::Subscriber planningFilterSub = n.subscribe("/kcl_rosplan/plan_filter", 100, &KCL_rosplan::PlanFilter::planningFilterCallback, &(kb->plan_filter));
	ros::Subscriber missionFilterSub = n.subscribe("/kcl_rosplan/plan_filter", 100, &KCL_rosplan::PlanFilter::missionFilterCallback, &(kb->plan_filter));

	ROS_INFO("KCL: (KB) Ready to receive");
	ros::spin();
	delete kb;
	return 0;
}
