//
// Created by martin on 02/04/19.
//

#include <iostream>
#include <string>
#include <vector>
#include <typeinfo>
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

GTEST_TEST(KnowledgeBaseTests, Test6_domain_operator_details_load_truck) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/rosplan_knowledge_base/domain/operator_details");
    rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;

    srv.request.name = "load-truck";

    client1.call(srv);

    std::string detailed_operator_name = srv.response.op.formula.name;

    ASSERT_EQ(srv.request.name, detailed_operator_name);

}

GTEST_TEST(KnowledgeBaseTests, Test7_domain_predicate_details_path) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>("/rosplan_knowledge_base/domain/predicate_details");
    rosplan_knowledge_msgs::GetDomainPredicateDetailsService srv;

    srv.request.name = "path";

    client1.call(srv);

    std::string detailed_predicate_name = srv.response.predicate.name;

    ASSERT_EQ(srv.request.name, detailed_predicate_name);

}

// for report: test more thoroughly functions, predicates, operators. Deeper testing with operator_details and predicate_deatails. Update picture, missing service: predicate_details.

int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "KnowledgeBaseTests");


    return RUN_ALL_TESTS();
}
