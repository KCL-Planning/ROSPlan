//
// Created by martin on 02/04/19.
//

#include <iostream>
#include <string>
#include <typeinfo>
#include <gtest/gtest.h>
#include <ros/package.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rosplan_knowledge_base/KnowledgeBase.h"
#include "rosplan_knowledge_msgs/GetDomainNameService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorService.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"


//KnowledgeBase *knowledge_base_tester;

GTEST_TEST(KnowledgeBaseTests, Test1_domain_name) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainNameService>("/rosplan_knowledge_base/domain/name");
    rosplan_knowledge_msgs::GetDomainNameService srv;

    client1.call(srv);

    ASSERT_EQ("driverlog-simple", srv.response.domain_name);

}

GTEST_TEST(KnowledgeBaseTests, Test2_domain_functions) {

    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/rosplan_knowledge_base/domain/functions");
    rosplan_knowledge_msgs::GetAttributeService srv;

    client1.call(srv);

    //rosplan_knowledge_msgs::KnowledgeItem response = srv.response.attributes;

    std::cout << typeid(srv.response.attributes).name() << std::endl;


    ASSERT_EQ(1 , 0);
    //ASSERT_EQ("[]", srv.response.attributes);

}

/*
GTEST_TEST(KnowledgeBaseTests, Test3_domain_operators) {

    /*
    ros::NodeHandle n("~");

    ros::ServiceClient client1 = n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>("/rosplan_knowledge_base/domain/operators");
    rosplan_knowledge_msgs::GetDomainOperatorService srv;

    client1.call(srv);

    std::vector<std::string> expected_operator_names = srv.response.operators;



    std::vector<std::string> operator_names;
    knowledge_base_tester->getOperatorNames(operator_names);

    std::vector<std::string> expected_operator_names = {"goto_waypoint", "localise", "dock", "undock", \
        "wait_load_at_machine", "switch_machine_on", "wait_unload", "ditch"};




    ASSERT_EQ(operator_names , expected_operator_names);


}
*/

int main(int argc, char **argv) {

    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "KnowledgeBaseTests");

    // knowledge_base_tester = new KnowledgeBase();

    return RUN_ALL_TESTS();
}
