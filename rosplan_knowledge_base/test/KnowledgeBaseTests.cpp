//
// Created by martin on 02/04/19.
//

#include <iostream>
#include <string>
#include <vector>
#include <gtest/gtest.h>
#include <ros/package.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rosplan_knowledge_base/KnowledgeBase.h"
#include "rosplan_knowledge_msgs/GetDomainNameService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetMetricService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"





GTEST_TEST(KnowledgeBaseTests, Test1_domain_name) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainNameService>("/rosplan_knowledge_base/domain/name");
    rosplan_knowledge_msgs::GetDomainNameService srv;

    client1.call(srv);

    ASSERT_EQ("driverlog-simple", srv.response.domain_name);

}



GTEST_TEST(KnowledgeBaseTests, Test2_domain_functions) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/rosplan_knowledge_base/domain/functions");
    rosplan_knowledge_msgs::GetDomainAttributeService srv;

    client1.call(srv);

    // Amazon-domain does not contain any functions

    ASSERT_EQ(0, srv.response.items.size());

}

GTEST_TEST(KnowledgeBaseTests, Test3_domain_predicates) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>("/rosplan_knowledge_base/domain/predicates");
    rosplan_knowledge_msgs::GetDomainAttributeService srv;

    std::vector<std::string> expected_predicate_names = {"at", "in", "link", "path"};

    client1.call(srv);

    std::vector<std::string> actual_predicate_names;

    for (auto rit=srv.response.items.begin(); rit!=srv.response.items.end(); rit++) {
        actual_predicate_names.push_back(rit->name);
    }

    ASSERT_EQ(expected_predicate_names, actual_predicate_names);

}


GTEST_TEST(KnowledgeBaseTests, Test4_domain_operators) {


    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>("/rosplan_knowledge_base/domain/operators");
    rosplan_knowledge_msgs::GetDomainOperatorService srv;

    std::vector<std::string> expected_operator_names = {"load-truck", "unload-truck", "board-truck", "get-out", "drive-truck", "walk"};

    client1.call(srv);

    std::vector<std::string> actual_operator_names;

    for (auto rit=srv.response.operators.begin(); rit!=srv.response.operators.end(); rit++) {
        actual_operator_names.push_back(rit->name);
    }

    ASSERT_EQ(expected_operator_names, actual_operator_names);

}

GTEST_TEST(KnowledgeBaseTests, Test5_domain_types){

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>("/rosplan_knowledge_base/domain/types");
    rosplan_knowledge_msgs::GetDomainTypeService srv;

    std::vector<std::string> expected_type_names = {"place", "locatable", "driver", "truck", "item"};
    std::vector<std::string> expected_super_type_names = {"object", "object", "locatable", "locatable", "locatable"};

    client1.call(srv);

    std::vector<std::string> actual_type_names = srv.response.types;
    std::vector<std::string> actual_super_type_names = srv.response.super_types;

    ASSERT_EQ(expected_type_names, actual_type_names);
    ASSERT_EQ(expected_super_type_names, actual_super_type_names);

}

GTEST_TEST(KnowledgeBaseTests, Test6_domain_operator_details) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/rosplan_knowledge_base/domain/operator_details");
    rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;

    srv.request.name = "load-truck";

    client1.call(srv);

    std::string detailed_operator_name = srv.response.op.formula.name;

    ASSERT_EQ(srv.request.name, detailed_operator_name);

}

GTEST_TEST(KnowledgeBaseTests, Test7_domain_predicate_details) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>("/rosplan_knowledge_base/domain/predicate_details");
    rosplan_knowledge_msgs::GetDomainPredicateDetailsService srv;

    srv.request.name = "path";

    client1.call(srv);

    std::string detailed_predicate_name = srv.response.predicate.name;

    ASSERT_EQ(srv.request.name, detailed_predicate_name);

}

GTEST_TEST(KnowledgeBaseTests, Test8_state_instances){

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/rosplan_knowledge_base/state/instances");
    rosplan_knowledge_msgs::GetInstanceService srv;

    std::vector<std::string> expected_instance_names = {"home", "amazon", "london", "myhouse"};

    srv.request.type_name = "place";

    client1.call(srv);

    std::vector<std::string> actual_instance_names = srv.response.instances;

    ASSERT_EQ(expected_instance_names, actual_instance_names);

}

GTEST_TEST(KnowledgeBaseTests, Test9_state_functions){

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/functions");
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = "link";

    client1.call(srv);

    // Amazon-domain does not contain any functions

    ASSERT_EQ(0, srv.response.attributes.size());

}

GTEST_TEST(KnowledgeBaseTests, Test10_state_propositions){

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/propositions");
    rosplan_knowledge_msgs::GetAttributeService srv;

    // Expected to be 4 propositions with name matching "link"
    std::vector<std::string> expected_propositions_names = {"link", "link", "link", "link"};

    srv.request.predicate_name = "link";

    client1.call(srv);

    std::vector<std::string> actual_propositions_names;

    for (auto rit=srv.response.attributes.begin(); rit!=srv.response.attributes.end(); rit++) {
        actual_propositions_names.push_back(rit->attribute_name);
    }

    ASSERT_EQ(expected_propositions_names, actual_propositions_names);

}

GTEST_TEST(KnowledgeBaseTests, Test11_state_goals){

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/goals");
    rosplan_knowledge_msgs::GetAttributeService srv;

    //Regardless of request data the response should be the same.
    srv.request.predicate_name = "at";
    client1.call(srv);
    ASSERT_EQ("at", srv.response.attributes[0].attribute_name);

    srv.request.predicate_name = "";
    client1.call(srv);
    ASSERT_EQ("at", srv.response.attributes[0].attribute_name);

    srv.request.predicate_name = "dgfhfhjfgh";
    client1.call(srv);
    ASSERT_EQ("at", srv.response.attributes[0].attribute_name);

}

GTEST_TEST(KnowledgeBaseTests, Test12_state_metric){

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetMetricService>("/rosplan_knowledge_base/state/metric");
    rosplan_knowledge_msgs::GetMetricService srv;

    client1.call(srv);

    // Amazon-problem does not have any metric

    ASSERT_EQ("", srv.response.metric.attribute_name);

}

GTEST_TEST(KnowledgeBaseTests, Test13_state_til){

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/timed_knowledge");
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = "at";

    client1.call(srv);

    // Amazon-problem does not have timed-initial-literals(TILs).

    ASSERT_EQ(0, srv.response.attributes.size());

}

GTEST_TEST(KnowledgeBaseTests, Test14_clear){

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/state/goals");
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = "anystring";
    client1.call(srv);
    // Check that goal attribute_name is "at"
    ASSERT_EQ("at", srv.response.attributes[0].attribute_name);

    ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("/rosplan_knowledge_base/clear");
    std_srvs::Empty emptySrv;

    client2.call(emptySrv);

    client1.call(srv);
    // Check that no goal exists after knowledge base is cleared
    ASSERT_EQ(0, srv.response.attributes.size());

}

//write in report that after knowledge base is cleared fruther tests are affected

GTEST_TEST(KnowledgeBaseTests, Test15_update){

    ros::NodeHandle n("~");

    ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("/rosplan_knowledge_base/clear");
    std_srvs::Empty emptySrv;

    client2.call(emptySrv);

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/rosplan_knowledge_base/update");
    rosplan_knowledge_msgs::KnowledgeUpdateService srv;

    rosplan_knowledge_msgs::KnowledgeItem knowledgeItem;

    knowledgeItem.knowledge_type = 0;
    knowledgeItem.instance_type = "driver";
    knowledgeItem.instance_name = "Martin";


    srv.request.update_type = 0;
    srv.request.knowledge = knowledgeItem;
    // ADD new knowledge
    client1.call(srv);

    ASSERT_EQ(true, srv.response.success);

    ros::ServiceClient client3 = n.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/rosplan_knowledge_base/state/instances");
    rosplan_knowledge_msgs::GetInstanceService instanceSrv;

    std::vector<std::string> expected_added_instance = {"Martin"};

    instanceSrv.request.type_name = "driver";

    client3.call(instanceSrv);

    ASSERT_EQ(expected_added_instance, instanceSrv.response.instances);

}

GTEST_TEST(KnowledgeBaseTests, Test16_update_array){

    ros::NodeHandle n("~");

    ros::ServiceClient client2 = n.serviceClient<std_srvs::Empty>("/rosplan_knowledge_base/clear");
    std_srvs::Empty emptySrv;

    client2.call(emptySrv);

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>("/rosplan_knowledge_base/update_array");
    rosplan_knowledge_msgs::KnowledgeUpdateServiceArray srv;

    rosplan_knowledge_msgs::KnowledgeItem knowledgeItem1;
    rosplan_knowledge_msgs::KnowledgeItem knowledgeItem2;
    rosplan_knowledge_msgs::KnowledgeItem knowledgeItem3;

    knowledgeItem1.knowledge_type = 0;
    knowledgeItem1.instance_type = "driver";
    knowledgeItem1.instance_name = "Martin";

    knowledgeItem2.knowledge_type = 0;
    knowledgeItem2.instance_type = "place";
    knowledgeItem2.instance_name = "home";

    knowledgeItem3.knowledge_type = 0;
    knowledgeItem3.instance_type = "place";
    knowledgeItem3.instance_name = "london";

    srv.request.update_type = {0, 0, 0};
    srv.request.knowledge = {knowledgeItem1, knowledgeItem2, knowledgeItem3};

    client1.call(srv);

    ASSERT_EQ(true, srv.response.success);


}

GTEST_TEST(KnowledgeBaseTests, Test17_query){

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/rosplan_knowledge_base/query_state");
    rosplan_knowledge_msgs::KnowledgeQueryService srv;

    rosplan_knowledge_msgs::KnowledgeItem knowledgeItem1;
    rosplan_knowledge_msgs::KnowledgeItem knowledgeItem2;
    rosplan_knowledge_msgs::KnowledgeItem knowledgeItem3;

    knowledgeItem1.knowledge_type = 0;
    knowledgeItem1.instance_type = "driver";
    knowledgeItem1.instance_name = "Martin";

    knowledgeItem2.knowledge_type = 0;
    knowledgeItem2.instance_type = "place";
    knowledgeItem2.instance_name = "home";

    knowledgeItem3.knowledge_type = 0;
    knowledgeItem3.instance_type = "driver";
    knowledgeItem3.instance_name = "Jonas";

    srv.request.knowledge = {knowledgeItem1, knowledgeItem2, knowledgeItem3};

    client1.call(srv);

    ASSERT_EQ(false, srv.response.all_true);

    std::vector<bool> expected_results = {true, true, false};

    std::vector<bool> actual_results;

    for (auto rit=srv.response.results.begin(); rit!=srv.response.results.end(); rit++) {
        actual_results.push_back(*rit);
    }

    ASSERT_EQ(expected_results, actual_results);

    ASSERT_EQ("Jonas", srv.response.false_knowledge[0].instance_name);

}

// for report: test more thoroughly functions, predicates, operators. Deeper testing with operator_details and predicate_deatails. Update picture, missing service: predicate_details.

int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "KnowledgeBaseTests");


    return RUN_ALL_TESTS();
}
