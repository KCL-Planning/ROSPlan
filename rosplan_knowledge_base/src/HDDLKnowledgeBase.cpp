#include "rosplan_knowledge_base/HDDLKnowledgeBase.h"

namespace KCL_rosplan {
    HDDLKnowledgeBase::HDDLKnowledgeBase(ros::NodeHandle& n) : KnowledgeBase(n), problem_file_path_(std::string("")) {}

    void HDDLKnowledgeBase::parseDomain(const std::string &domain_file_path, const std::string &problem_file_path) {

        problem_file_path_ = problem_file_path;

        ROS_INFO("KCL: Parsing hddl domain : %s", domain_file_path.c_str());
        hddl_parser_.parse(domain_file_path);

        if (!hddl_parser_.parsing_ok()) {
            ROS_ERROR("KCL: (%s) There were syntax errors in the domain file.",
                      ros::this_node::getName().c_str());
            ros::shutdown();
        }

        if (problem_file_path_ != "") addInitialState();
        ROS_INFO("KCL: successfully parsed hddl domain!");
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
//         std::vector<std::string> robots;
//         robots.push_back("tiago");
//         model_instances.insert(std::pair<std::string, std::vector<std::string>>("robot", robots));
        ROS_WARN("Initial state not yet implemented, cannot load: %s", problem_file_path_.c_str());
    }

    void HDDLKnowledgeBase::addConstants() {}

    bool HDDLKnowledgeBase::getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request  &req, rosplan_knowledge_msgs::GetDomainNameService::Response &res) {
        res.domain_name = hddl_parser_.domain_.domain_name_;
        return true;
    }

    bool HDDLKnowledgeBase::getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res) {
        for(auto it = hddl_parser_.domain_.domain_types_.begin(); it!=hddl_parser_.domain_.domain_types_.end(); it++) {
            res.types.push_back(it->first);
            res.super_types.push_back(it->second);
        }
        return true;
    }

    bool HDDLKnowledgeBase::getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {

        for(auto it=hddl_parser_.domain_.domain_predicates_.begin(); it!=hddl_parser_.domain_.domain_predicates_.end(); it++) {

            rosplan_knowledge_msgs::DomainFormula df;
            diagnostic_msgs::KeyValue kv;

            df.name =  it->name;
            for(auto pit=it->pred_params.params.begin(); pit!=it->pred_params.params.end(); pit++) {
                kv.key =  *pit;
                kv.value = it->pred_params.params_map[*pit];
                df.typed_parameters.push_back(kv);
            }
            res.items.push_back(df);
        }

        return true;
    }

    bool HDDLKnowledgeBase::getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) {

        for(auto it=hddl_parser_.domain_.domain_predicates_.begin(); it!=hddl_parser_.domain_.domain_predicates_.end(); it++) {

            if(it->name == req.name) {

                rosplan_knowledge_msgs::DomainFormula df;
                diagnostic_msgs::KeyValue kv;

                df.name =  it->name;
                for(auto pit=it->pred_params.params.begin(); pit!=it->pred_params.params.end(); pit++) {
                    kv.key =  *pit;
                    kv.value = it->pred_params.params_map[*pit];
                    df.typed_parameters.push_back(kv);
                }
                res.predicate = df;
            }
        }

        return true;
    }

    bool HDDLKnowledgeBase::getFunctionPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {
        ROS_WARN("Functions are not supported in hddl, returning empty response");
        return false;
    }

    bool HDDLKnowledgeBase::getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res) {
        ROS_WARN("Not implemented yet (getOperators)");
        return false;
    }

    bool HDDLKnowledgeBase::getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) {
        ROS_WARN("Not implemented yet! (getOperatorDetails)");
        return false;
    }
}
