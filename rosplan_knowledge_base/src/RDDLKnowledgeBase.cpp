
#include <rosplan_knowledge_base/RDDLKnowledgeBase.h>

#include "rosplan_knowledge_base/RDDLKnowledgeBase.h"


namespace KCL_rosplan {

    /*-----------------*/
    /* fetching domain */
    /*-----------------*/

    /* get domain name */
    bool RDDLKnowledgeBase::getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request &req,
                                                       rosplan_knowledge_msgs::GetDomainNameService::Response &res) {
        if (!domain_parser.domain_parsed) return false;
        res.domain_name = domain_parser.domain_name;
        return true;
    }

    /* types */
    bool RDDLKnowledgeBase::getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request &req,
                                     rosplan_knowledge_msgs::GetDomainTypeService::Response &res) {
        if (!domain_parser.domain_parsed) return false;

        // Iterate over types
        std::set<std::string> types;
        std::set<std::string> super_types;
        for (auto it = domain_parser.rddlTask->types.begin() ; it != domain_parser.rddlTask->types.end(); ++it) {
            // FIXME should check if a type is a already supertype so we have disjunct sets?
            // FIXME filter out basic types?
            types.insert(it->first);
            if (it->second->superType) super_types.insert(it->second->superType->name);
        }

        res.types.assign(types.begin(), types.end());
        res.super_types.assign(types.begin(), types.end());
        return true;
    }

	/* get domain predicates */
    bool RDDLKnowledgeBase::getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request &req,
                                          rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {
        // FIXME I consider a predicate any state-fluent or non-fluent with a bool type
        // FIXME what about interm fluents and observ-fluents? Not sure if supported by the parser
        for (auto it = domain_parser.rddlTask->variableDefinitions.begin() ;
                  it != domain_parser.rddlTask->variableDefinitions.end() ; ++it) {

            // Check if the parametrized variables (pVariables) are predicates
            if ((it->second->variableType != ParametrizedVariable::STATE_FLUENT and
                it->second->variableType != ParametrizedVariable::NON_FLUENT) or it->second->valueType->name != "bool") {
                continue; // Skip
            }
            // predicate name
            rosplan_knowledge_msgs::DomainFormula formula;
            formula.name = it->second->variableName;

            // predicate variables
            formula.typed_parameters = getTypedParams(it->second->params);
            res.items.push_back(formula);
        }
        return true;
    }

    /* get domain functions */
    bool RDDLKnowledgeBase::getFunctionPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request &req,
                                                  rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {
        // FIXME I consider a function any state-fluent or non-fluent with a numeric type (real/int)
        // FIXME what about interm fluents and observ-fluents? Not sure if supported by the parser
        for (auto it = domain_parser.rddlTask->variableDefinitions.begin() ;
             it != domain_parser.rddlTask->variableDefinitions.end() ; ++it) {

            // Check if the parametrized variables (pVariables) are functions
            if ((it->second->variableType != ParametrizedVariable::STATE_FLUENT and
                 it->second->variableType != ParametrizedVariable::NON_FLUENT) or
                 (it->second->valueType->name != "int" and  it->second->valueType->name != "real")) continue; // Skip

             // function name
            rosplan_knowledge_msgs::DomainFormula formula;
            formula.name = it->second->variableName;

            // function parameters
            formula.typed_parameters = getTypedParams(it->second->params);
            res.items.push_back(formula);
        }
        return true;
    }

    /* get domain operators */
    bool RDDLKnowledgeBase::getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request &req,
                                         rosplan_knowledge_msgs::GetDomainOperatorService::Response &res) {
        // TODO
        return false;
    }

    /* get domain operator details */
    bool RDDLKnowledgeBase::getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request &req,
                                               rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) {
        // TODO
        // FIXME precompute them? As computing them might be hard...
        return false;
    }

    /* get domain predicate details */
    bool RDDLKnowledgeBase::getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request &req,
                                                 rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) {
        auto it = domain_parser.rddlTask->variableDefinitions.find(req.name);
        if (it != domain_parser.rddlTask->variableDefinitions.end()) {
            res.predicate.name = it->second->variableName;

            // predicate parameters
            res.predicate.typed_parameters = getTypedParams(it->second->params);
            return true;
        }
        ROS_WARN("KCL: (%s) Unknown predicate \"%s\".", ros::this_node::getName().c_str(), req.name.c_str());
        return false;
    }

    /*-------------------------------------*/
    /* add initial state to knowledge base */
    /*-------------------------------------*/
    /* get the initial state from the domain and problem files */
    void RDDLKnowledgeBase::addInitialState() {

        model_instances = getInstances(domain_parser.rddlTask->objects);


        /*
		model_facts = problem_visitor.returnFacts();
		model_functions = problem_visitor.returnFunctions();
		model_goals = problem_visitor.returnGoals();
		model_timed_initial_literals = problem_visitor.returnTimedKnowledge();
		if (problem->metric) {
			model_metric = problem_visitor.returnMetric();
		}
         */
    }



    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Private methods

    /* Get ROS-like typed parameters */
    std::vector<diagnostic_msgs::KeyValue> RDDLKnowledgeBase::getTypedParams(const std::vector<Parameter *> &params) {
        std::vector<diagnostic_msgs::KeyValue> ros_params;
        for (auto pit = params.begin(); pit != params.end(); ++pit) {
            diagnostic_msgs::KeyValue param;
            param.key = (*pit)->name; // Parameter name
            size_t pos = param.key.find('?');
            if (pos != std::string::npos) param.key.erase(pos, 1); // Remove the ? if present
            param.value = (*pit)->type->name; // Type name
            ros_params.push_back(param);
        }
        return ros_params;
    }

    /* Get object instances */
    std::map<std::string, std::vector<std::string> > RDDLKnowledgeBase::getInstances(const std::map<std::string, Object*>& instance_objects) {
        map<string, std::vector<string>> instances;
        for (auto it = instance_objects.begin(); it != instance_objects.end(); ++it) {
            if (it->first == "false" or it->first == "true") continue; // Skip the "true" and "false" objects
            instances[it->second->type->name].push_back(it->first);
        }
        return instances;
    }


}