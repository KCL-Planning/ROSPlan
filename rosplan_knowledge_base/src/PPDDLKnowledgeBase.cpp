//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 30/09/18.
//

#include <rosplan_knowledge_base/PPDDLKnowledgeBase.h>

#include "rosplan_knowledge_base/PPDDLKnowledgeBase.h"

void KCL_rosplan::PPDDLKnowledgeBase::parseDomain(const std::string &domain_file_path,
                                                  const std::string &problem_file_path) {
    PPDDLDomainPtr t = domain_parser.parseDomainProblem(domain_file_path, problem_file_path); // The parser stores the task
    if (t == nullptr) {
        ROS_ERROR("KCL: (%s) There were syntax errors in the domain or problem file.", ros::this_node::getName().c_str());
        ros::shutdown();
    }
    if (problem_file_path != "") addInitialState();

}

bool KCL_rosplan::PPDDLKnowledgeBase::getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request &req,
                                                    rosplan_knowledge_msgs::GetDomainNameService::Response &res) {
    if (not domain_parser.domain_parsed) return false;
    res.domain_name = domain_parser.domain_name;
    return true;
}

std::vector<diagnostic_msgs::KeyValue>
KCL_rosplan::PPDDLKnowledgeBase::getTypedParams(const ppddl_parser::TypeList& params) {
    std::vector<diagnostic_msgs::KeyValue> ret;
    std::map<std::string, int> var_names;
    for (auto it = params.begin(); it != params.end(); ++it) {
        diagnostic_msgs::KeyValue param;
        param.value = domain_parser.domain->types().typestring(*it); // type name

        std::string vname = param.value.substr(0, 1);
        if (var_names.find(vname) != var_names.end()) var_names[vname] = 0;
        else {
            ++var_names[vname];
            vname += std::to_string(var_names[vname]);
        }

        param.key = vname;
    }
    return ret;
}

bool KCL_rosplan::PPDDLKnowledgeBase::getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request &req,
                                               rosplan_knowledge_msgs::GetDomainTypeService::Response &res) {
    if (not domain_parser.domain_parsed) return false;
    res.types=domain_parser.domain->types().names();
    return true;
}

bool KCL_rosplan::PPDDLKnowledgeBase::getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request &req,
                                                    rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {
    if (not domain_parser.domain_parsed) return false;
    const std::vector<std::string> names = domain_parser.domain->predicates().names();
    for (auto it = names.begin(); it != names.end(); ++it) {
        rosplan_knowledge_msgs::DomainFormula formula;
        formula.name = *it;

        // predicate variables
        auto pred = domain_parser.domain->predicates().find_predicate(*it);
        formula.typed_parameters = getTypedParams(domain_parser.domain->predicates().parameters(*pred));
        res.items.push_back(formula);
    }
    return true;
}


bool KCL_rosplan::PPDDLKnowledgeBase::getPredicateDetails(
        rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request &req,
        rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) {
    if (not domain_parser.domain_parsed) return false;
    const std::vector<std::string> names = domain_parser.domain->predicates().names();
    for (auto it = names.begin(); it != names.end(); ++it) {
        if (*it == req.name) {
            res.predicate.name = *it;

            // predicate variables
            auto pred = domain_parser.domain->predicates().find_predicate(*it);
            res.predicate.typed_parameters = getTypedParams(domain_parser.domain->predicates().parameters(*pred));
            return true;
        }
    }
    return false;
}


bool
KCL_rosplan::PPDDLKnowledgeBase::getFunctionPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request &req,
                                                       rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {
    if (not domain_parser.domain_parsed) return false;
    const std::vector<std::string> names = domain_parser.domain->functions().names();
    for (auto it = names.begin(); it != names.end(); ++it) {
        rosplan_knowledge_msgs::DomainFormula formula;
        formula.name = *it;

        auto f = domain_parser.domain->functions().find_function(*it);
        formula.typed_parameters = getTypedParams(domain_parser.domain->functions().parameters(*f));
    }
    return true;
}

bool KCL_rosplan::PPDDLKnowledgeBase::getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request &req,
                                                   rosplan_knowledge_msgs::GetDomainOperatorService::Response &res) {
    if (not domain_parser.domain_parsed) return false;
    for (auto it = domain_parser.domain->actions().begin(); it != domain_parser.domain->actions().end(); ++it) {
        rosplan_knowledge_msgs::DomainFormula action;
        action.name = it->second->name();
        std::map<std::string, int> var_names;
        for (auto pit = it->second->parameters().begin(); pit != it->second->parameters().end(); ++pit) {
            diagnostic_msgs::KeyValue param;
            param.value = ppddl_parser::TypeTable::typestring(ppddl_parser::TermTable::type(*pit)); // type name

            std::string vname = param.value.substr(0, 1);
            if (var_names.find(vname) != var_names.end()) var_names[vname] = 0;
            else {
                ++var_names[vname];
                vname += std::to_string(var_names[vname]);
            }

            param.key = vname;
            action.typed_parameters.push_back(param);
        }
        res.operators.push_back(action);
    }
    return true;

}
