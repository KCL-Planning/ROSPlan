#include "rosplan_knowledge_base/HDDLKnowledgeBase.h"

namespace KCL_rosplan {
//     HDDLKnowledgeBase (ros::NodeHandle& n) {
//         
//     }

    void HDDLKnowledgeBase::parseDomain(const std::string &domain_file_path, const std::string &problem_file_path) {
//         domain_path_ = domain_file_path;

        // TODO parse HDDL domain
//         if (?) {
//             ROS_ERROR("OTPEL: (%s) There were syntax errors in the domain or instance file.",
//                       ros::this_node::getName().c_str());
//             ros::shutdown();
//         }

        if (problem_file_path != "")
            addInitialState();
        ROS_INFO("Parsing the domain...");
    }

    void HDDLKnowledgeBase::addInitialState() {
//         VALVisitorProblem problem_visitor(domain_parser.domain, problem_parser.problem);
//         model_instances = problem_visitor.returnInstances();
//         model_facts = problem_visitor.returnFacts();
//         model_functions = problem_visitor.returnFunctions();
//         model_goals = problem_visitor.returnGoals();
//         model_timed_initial_literals = problem_visitor.returnTimedKnowledge();
//         if (problem_parser.problem->metric) {
//             model_metric = problem_visitor.returnMetric();
//         }

        // model instances -> std::map<std::string, std::vector<std::string> >
        std::vector<std::string> robots;
        robots.push_back("tiago");
        model_instances.insert(std::pair<std::string, std::vector<std::string>>("robot", robots));
    }

    void HDDLKnowledgeBase::addConstants() {}

    bool HDDLKnowledgeBase::getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request  &req, rosplan_knowledge_msgs::GetDomainNameService::Response &res) {
        res.domain_name = "turtlebot"; // TODO: set proper name from hddl file
        return true;
    }

    bool HDDLKnowledgeBase::getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res) {
        res.types.push_back("waypoint");
        res.types.push_back("robot");
//      res.super_types
        return true;
    }

    bool HDDLKnowledgeBase::getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {
        rosplan_knowledge_msgs::DomainFormula df;
        diagnostic_msgs::KeyValue kv;
        
        df.name =  "robot_at";
        kv.key =  "v";
        kv.value =  "robot";
        df.typed_parameters.push_back(kv);
        kv.key =  "wp";
        kv.value =  "waypoint";
        df.typed_parameters.push_back(kv);
        res.items.push_back(df);
        df.typed_parameters.clear();
        
        df.name =  "visited";
        kv.key =  "wp";
        kv.value =  "waypoint";
        df.typed_parameters.push_back(kv);
        res.items.push_back(df);
        df.typed_parameters.clear();
        
        df.name =  "undocked";
        kv.key =  "v";
        kv.value =  "robot";
        df.typed_parameters.push_back(kv);
        res.items.push_back(df);
        df.typed_parameters.clear();
        
        df.name =  "docked";
        kv.key =  "v";
        kv.value =  "robot";
        df.typed_parameters.push_back(kv);
        res.items.push_back(df);
        df.typed_parameters.clear();
        
        df.name =  "localised";
        kv.key =  "v";
        kv.value =  "robot";
        df.typed_parameters.push_back(kv);
        res.items.push_back(df);
        df.typed_parameters.clear();
        
        df.name =  "dock_at";
        kv.key =  "wp";
        kv.value =  "waypoint";
        df.typed_parameters.push_back(kv);
        res.items.push_back(df);
        df.typed_parameters.clear();
        
        return true;
    }

    bool HDDLKnowledgeBase::getFunctionPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {
        return true;
    }

    bool HDDLKnowledgeBase::getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res) {
        return true;
    }

    bool HDDLKnowledgeBase::getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) {
        return true;
    }

    bool HDDLKnowledgeBase::getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) {
        return true;
    }
}
