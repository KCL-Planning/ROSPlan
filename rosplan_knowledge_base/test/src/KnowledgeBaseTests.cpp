/**
 * 
 * Copyright [2019] <KCL King's College London>
 * 
 * Author: Martin Koling (martinh.koling@kcl.ac.uk)
 * Author: Oscar Lima (oscar.lima@dfki.de)
 * 
 * Unit tests for ROSPlan knowledge base
 */

#include <ros/ros.h>
#include <string>
#include <vector>
#include <gtest/gtest.h>

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
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"

GTEST_TEST(KnowledgeBaseTests, Test1_domain_name) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/domain/name";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetDomainNameService>(srv_name);
    rosplan_knowledge_msgs::GetDomainNameService srv;

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    ASSERT_EQ("driverlog-simple", srv.response.domain_name);
}

GTEST_TEST(KnowledgeBaseTests, Test2_domain_functions) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/domain/functions";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(srv_name);
    rosplan_knowledge_msgs::GetDomainAttributeService srv;

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    // Amazon-domain does not contain any functions
    EXPECT_FALSE(srv.response.items.size());
}

GTEST_TEST(KnowledgeBaseTests, Test3_domain_predicates) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/domain/predicates";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(srv_name);
    rosplan_knowledge_msgs::GetDomainAttributeService srv;

    std::vector<std::string> expected_predicate_names = {"at", "in", "link", "path"};

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    std::vector<std::string> actual_predicate_names;

    for (auto rit=srv.response.items.begin(); rit!=srv.response.items.end(); rit++) {
        actual_predicate_names.push_back(rit->name);
    }

    ASSERT_EQ(expected_predicate_names, actual_predicate_names);
}

GTEST_TEST(KnowledgeBaseTests, Test4_domain_operators) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/domain/operators";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>(srv_name);
    rosplan_knowledge_msgs::GetDomainOperatorService srv;

    std::vector<std::string> expected_operator_names = {"load-truck", "unload-truck", "board-truck", "get-out", "drive-truck", "walk"};

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    std::vector<std::string> actual_operator_names;

    for (auto rit=srv.response.operators.begin(); rit!=srv.response.operators.end(); rit++) {
        actual_operator_names.push_back(rit->name);
    }

    ASSERT_EQ(expected_operator_names, actual_operator_names);
}

GTEST_TEST(KnowledgeBaseTests, Test5_domain_types){

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/domain/types";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>(srv_name);
    rosplan_knowledge_msgs::GetDomainTypeService srv;

    std::vector<std::string> expected_type_names = {"place", "locatable", "driver", "truck", "item"};
    std::vector<std::string> expected_super_type_names = {"object", "object", "locatable", "locatable", "locatable"};

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    std::vector<std::string> actual_type_names = srv.response.types;
    std::vector<std::string> actual_super_type_names = srv.response.super_types;

    ASSERT_EQ(expected_type_names, actual_type_names);
    ASSERT_EQ(expected_super_type_names, actual_super_type_names);
}

GTEST_TEST(KnowledgeBaseTests, Test6_domain_operator_details) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/domain/operator_details";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(srv_name);
    rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;

    srv.request.name = "load-truck";

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    std::string detailed_operator_name = srv.response.op.formula.name;

    ASSERT_EQ(srv.request.name, detailed_operator_name);
}

GTEST_TEST(KnowledgeBaseTests, Test7_domain_predicate_details) {

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/domain/predicate_details";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>(srv_name);
    rosplan_knowledge_msgs::GetDomainPredicateDetailsService srv;

    srv.request.name = "path";

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    std::string detailed_predicate_name = srv.response.predicate.name;

    ASSERT_EQ(srv.request.name, detailed_predicate_name);
}

GTEST_TEST(KnowledgeBaseTests, Test8_state_instances){

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/state/instances";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(srv_name);
    rosplan_knowledge_msgs::GetInstanceService srv;

    std::vector<std::string> expected_instance_names = {"home", "amazon", "london", "myhouse"};

    srv.request.type_name = "place";

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    std::vector<std::string> actual_instance_names = srv.response.instances;

    ASSERT_EQ(expected_instance_names, actual_instance_names);
}

GTEST_TEST(KnowledgeBaseTests, Test9_state_functions){

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/state/functions";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(srv_name);
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = "link";

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    // Amazon-domain does not contain any functions
    EXPECT_FALSE(srv.response.attributes.size());
}

GTEST_TEST(KnowledgeBaseTests, Test10_state_propositions){

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/state/propositions";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(srv_name);
    rosplan_knowledge_msgs::GetAttributeService srv;

    // Expected to be 4 propositions with name matching "link"
    std::vector<std::string> expected_propositions_names = {"link", "link", "link", "link"};

    srv.request.predicate_name = "link";

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    std::vector<std::string> actual_propositions_names;

    for (auto rit=srv.response.attributes.begin(); rit!=srv.response.attributes.end(); rit++) {
        actual_propositions_names.push_back(rit->attribute_name);
    }

    ASSERT_EQ(expected_propositions_names, actual_propositions_names);
}

GTEST_TEST(KnowledgeBaseTests, Test11_state_goals){

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/state/goals";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(srv_name);
    rosplan_knowledge_msgs::GetAttributeService srv;

    //Regardless of request data the response should be the same.
    srv.request.predicate_name = "at";
    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);
    ASSERT_EQ("at", srv.response.attributes[0].attribute_name);

    srv.request.predicate_name = "";
    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);
    ASSERT_EQ("at", srv.response.attributes[0].attribute_name);
    // check for size of arguments to be equal to a known value (there is 1 goal in the problem file)
    EXPECT_TRUE(srv.response.attributes.size());

    srv.request.predicate_name = "dgfhfhjfgh";
    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);
    ASSERT_EQ("at", srv.response.attributes[0].attribute_name);
}

GTEST_TEST(KnowledgeBaseTests, Test12_state_metric){

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/state/metric";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetMetricService>(srv_name);
    rosplan_knowledge_msgs::GetMetricService srv;

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    // Amazon-problem does not have any metric

    ASSERT_EQ("", srv.response.metric.attribute_name);
}

GTEST_TEST(KnowledgeBaseTests, Test13_state_til){

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/state/timed_knowledge";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(srv_name);
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = "at";

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    // Amazon-problem does not have timed-initial-literals(TILs).

    EXPECT_FALSE(srv.response.attributes.size());
}

GTEST_TEST(KnowledgeBaseTests, Test14_clear){

    ros::NodeHandle nh("~");

    std::string srv_name1 = "/rosplan_knowledge_base/state/goals";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(srv_name1);
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = "anystring";
    ros::service::waitForService(srv_name1, ros::Duration(1));
    client1.call(srv);
    // Check that goal attribute_name is "at"
    ASSERT_EQ("at", srv.response.attributes[0].attribute_name);

    std::string srv_name2 = "/rosplan_knowledge_base/clear";
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>(srv_name2);
    std_srvs::Empty emptySrv;

    ros::service::waitForService(srv_name2, ros::Duration(1));
    client2.call(emptySrv);

    ros::service::waitForService(srv_name1, ros::Duration(1));
    client1.call(srv);
    // Check that no goal exists after knowledge base is cleared
    EXPECT_FALSE(srv.response.attributes.size());
}

GTEST_TEST(KnowledgeBaseTests, Test15_update){

    ros::NodeHandle nh("~");

    std::string srv_name1 = "/rosplan_knowledge_base/clear";
    ros::ServiceClient client2 = nh.serviceClient<std_srvs::Empty>(srv_name1);
    std_srvs::Empty emptySrv;

    ros::service::waitForService(srv_name1, ros::Duration(1));
    client2.call(emptySrv);

    std::string srv_name2 = "/rosplan_knowledge_base/update";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>(srv_name2);
    rosplan_knowledge_msgs::KnowledgeUpdateService srv;

    rosplan_knowledge_msgs::KnowledgeItem knowledgeItem;

    knowledgeItem.knowledge_type = 0;
    knowledgeItem.instance_type = "driver";
    // knowledgeItem.instance_name = "Martin"; // TODO: uncomment when #199 is solved
    knowledgeItem.instance_name = "martin"; // TODO: delete once #199 is solved

    srv.request.update_type = 0;
    srv.request.knowledge = knowledgeItem;

    ros::service::waitForService(srv_name2, ros::Duration(1));
    client1.call(srv);

    EXPECT_TRUE(srv.response.success);

    std::string srv_name3 = "/rosplan_knowledge_base/state/instances";
    ros::ServiceClient client3 = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(srv_name3);
    rosplan_knowledge_msgs::GetInstanceService instanceSrv;

    std::vector<std::string> expected_added_instance = {"martin"};

    instanceSrv.request.type_name = "driver";

    ros::service::waitForService(srv_name3, ros::Duration(1));
    client3.call(instanceSrv);

    ASSERT_EQ(expected_added_instance, instanceSrv.response.instances);
}

GTEST_TEST(KnowledgeBaseTests, Test16_update_array){

    ros::NodeHandle nh("~");

    std::string srv_name1 = "/rosplan_knowledge_base/clear";
    ros::ServiceClient client1 = nh.serviceClient<std_srvs::Empty>(srv_name1);
    std_srvs::Empty emptySrv;

    ros::service::waitForService(srv_name1, ros::Duration(1));
    client1.call(emptySrv);

    std::string srv_name2 = "/rosplan_knowledge_base/update_array";
    ros::ServiceClient client2 = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(srv_name2);
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

    ros::service::waitForService(srv_name2, ros::Duration(1));
    client2.call(srv);

    EXPECT_TRUE(srv.response.success);
}

GTEST_TEST(KnowledgeBaseTests, Test17_query){

    ros::NodeHandle nh("~");

    std::string srv_name = "/rosplan_knowledge_base/query_state";
    ros::ServiceClient client1 = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>(srv_name);
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

    ros::service::waitForService(srv_name, ros::Duration(1));
    client1.call(srv);

    EXPECT_FALSE(srv.response.all_true);

    std::vector<bool> expected_results = {true, true, false};

    std::vector<bool> actual_results;

    for (auto rit=srv.response.results.begin(); rit!=srv.response.results.end(); rit++) {
        actual_results.push_back(*rit);
    }

    ASSERT_EQ(expected_results, actual_results);

    ASSERT_EQ("Jonas", srv.response.false_knowledge[0].instance_name);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {

    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "KnowledgeBaseTests");

    return RUN_ALL_TESTS();
}
