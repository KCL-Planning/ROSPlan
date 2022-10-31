
#include "rosplan_knowledge_base/PDDLKnowledgeBase.h"

namespace KCL_rosplan {

    /*-----------------*/
    /* fetching domain */
    /*-----------------*/

    /* get domain name */
    bool PDDLKnowledgeBase::getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request  &req, rosplan_knowledge_msgs::GetDomainNameService::Response &res) {
        if(!domain_parser.domain_parsed) return false;
        res.domain_name = domain_parser.domain->name;
        return true;
    }

    /* get domain types */
    bool PDDLKnowledgeBase::getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res) {

        if(!domain_parser.domain_parsed) return false;

        VAL1_2::pddl_type_list* types = domain_parser.domain->types;
        if (types)
        {
            for (VAL1_2::pddl_type_list::const_iterator ci = types->begin(); ci != types->end(); ci++) {
                const VAL1_2::pddl_type* type = *ci;
                res.types.push_back(type->getName());
                if(type->type) res.super_types.push_back(type->type->getName());
                else res.super_types.push_back("");
            }
        }
        return true;
    }

    /* get domain predicates */
    bool PDDLKnowledgeBase::getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {

        VAL1_2::pred_decl_list* predicates = domain_parser.domain->predicates;
        if(predicates) {
            for (VAL1_2::pred_decl_list::const_iterator ci = predicates->begin(); ci != predicates->end(); ci++) {
                const VAL1_2::pred_decl* predicate = *ci;

                // predicate name
                rosplan_knowledge_msgs::DomainFormula formula;
                formula.name = predicate->getPred()->symbol::getName();

                // predicate variables
                for (VAL1_2::var_symbol_list::const_iterator vi = predicate->getArgs()->begin(); vi != predicate->getArgs()->end(); vi++) {
                    const VAL1_2::var_symbol* var = *vi;
                    diagnostic_msgs::KeyValue param;
                    param.key = var->pddl_typed_symbol::getName();
                    if (var->type != nullptr) param.value = var->type->getName();
                    else param.value = "object";
                    formula.typed_parameters.push_back(param);
                }
                res.items.push_back(formula);
            }
        }
        return true;
    }

    /* get domain functions */
    bool PDDLKnowledgeBase::getFunctionPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {

        VAL1_2::func_decl_list* functions = domain_parser.domain->functions;
        if(functions) {
            for (VAL1_2::func_decl_list::const_iterator ci = functions->begin(); ci != functions->end(); ci++) {
                const VAL1_2::func_decl* function = *ci;

                // function name
                rosplan_knowledge_msgs::DomainFormula formula;
                formula.name = function->getFunction()->symbol::getName();

                // parameters
                for (VAL1_2::var_symbol_list::const_iterator vi = function->getArgs()->begin(); vi != function->getArgs()->end(); vi++) {
                    const VAL1_2::var_symbol* var = *vi;
                    diagnostic_msgs::KeyValue param;
                    param.key = var->pddl_typed_symbol::getName();
                    if (var->type != nullptr) param.value = var->type->getName();
                    else param.value = "object";
                    formula.typed_parameters.push_back(param);
                }
                res.items.push_back(formula);
            }
        }
        return true;
    }

    /* get domain operators */
    bool PDDLKnowledgeBase::getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res) {

        VAL1_2::operator_list* operators = domain_parser.domain->ops;
        for (VAL1_2::operator_list::const_iterator ci = operators->begin(); ci != operators->end(); ci++) {
            const VAL1_2::operator_* op = *ci;

            // name
            rosplan_knowledge_msgs::DomainFormula formula;
            formula.name = op->name->symbol::getName();

            // parameters
            for (VAL1_2::var_symbol_list::const_iterator vi = op->parameters->begin(); vi != op->parameters->end(); vi++) {
                const VAL1_2::var_symbol* var = *vi;
                diagnostic_msgs::KeyValue param;
                param.key = var->pddl_typed_symbol::getName();
                if (var->type != nullptr) param.value = var->type->getName();
                else param.value = "object";
                formula.typed_parameters.push_back(param);
            }

            res.operators.push_back(formula);
        }
        return true;
    }

    /* get domain operator details */
    bool PDDLKnowledgeBase::getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) {
        VALVisitorOperator op_visitor;
        VAL1_2::operator_list* operators = domain_parser.domain->ops;
        for (VAL1_2::operator_list::const_iterator ci = operators->begin(); ci != operators->end(); ci++) {
            if((*ci)->name->symbol::getName() == req.name) {
                op_visitor.visit_operator_(*ci);
                res.op = op_visitor.msg;
                return true;
            }
        }
        return false;
    }

    /* get domain predicate details */
    bool PDDLKnowledgeBase::getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) {
        VALVisitorPredicate pred_visitor;
        VAL1_2::pred_decl_list* predicates = domain_parser.domain->predicates;
        for (VAL1_2::pred_decl_list::const_iterator ci = predicates->begin(); ci != predicates->end(); ci++) {
            if((*ci)->getPred()->symbol::getName() == req.name) {
                pred_visitor.visit_pred_decl(*ci);
                res.predicate = pred_visitor.msg;

                // predicate is sensed
                if(sensed_predicates.find(req.name) == sensed_predicates.end()) {
                    sensed_predicates[req.name] = false;
                }
                res.is_sensed = sensed_predicates[req.name];

                return true;
            }
        }
        return false;
    }

    /*-------------------------------------*/
    /* add initial state to knowledge base */
    /*-------------------------------------*/

    /* get constants from the domain file */
    void PDDLKnowledgeBase::addConstants() {

        if(!domain_parser.domain_parsed) return;

        VAL1_2::const_symbol_list *c = domain_parser.domain->constants;
        if (c)
        {
            for (VAL1_2::const_symbol_list::const_iterator symbolListIterator = c->begin();
                 symbolListIterator != c->end(); symbolListIterator++) {
                const VAL1_2::const_symbol *object = *symbolListIterator;
                domain_constants[object->type->getName()].push_back(object->pddl_typed_symbol::getName());
            }
        }
    }

    /* get the initial state from the domain and problem files */
    void PDDLKnowledgeBase::addInitialState() {
        VALVisitorProblem problem_visitor(domain_parser.domain, problem_parser.problem);
        model_instances = problem_visitor.returnInstances();
        model_facts = problem_visitor.returnFacts();
        model_functions = problem_visitor.returnFunctions();
        model_goals = problem_visitor.returnGoals();
        model_timed_initial_literals = problem_visitor.returnTimedKnowledge();
        if (problem_parser.problem->metric) {
            model_metric = problem_visitor.returnMetric();
        }
    }

    void PDDLKnowledgeBase::parseDomain(const std::string &domain_file_path, const std::string &problem_file_path) {
        std::ifstream file_check;
        VAL1_2::analysis* an_analysis = new VAL1_2::analysis;
        domain_parser.val_analysis = an_analysis;
        problem_parser.val_analysis = an_analysis;

        // parse domain
        ROS_INFO("KCL: (%s) Parsing domain", ros::this_node::getName().c_str());
        domain_parser.domain_parsed = false;
        VAL1_2::domain* domain;
        file_check.open(domain_file_path.c_str());
        if(!file_check.good()) {
            ROS_ERROR("KCL: (%s) Domain file does not exist : (%s).",
                      ros::this_node::getName().c_str(), domain_file_path.c_str());
            ros::shutdown();
        } else {
            file_check.close();
            domain = domain_parser.parseDomain(domain_file_path);
            if(domain) {
                addConstants();
            } else {
                ROS_ERROR("KCL: (%s) There were syntax errors in the domain file.", ros::this_node::getName().c_str());
                ros::shutdown();
            }
        }

        // parse problem and add initial state
        if(problem_file_path != "") {
            ROS_INFO("KCL: (%s) Parsing initial state", ros::this_node::getName().c_str());
            problem_parser.problem_parsed = false;
            file_check.open(problem_file_path.c_str());
            if(!file_check.good()) {
                ROS_WARN("KCL: (%s) Initial state file does not exist.", ros::this_node::getName().c_str());
            } else {
                file_check.close();
                VAL1_2::problem* problem = problem_parser.parseProblem(problem_file_path);
                if(problem) {
                    addInitialState();
                } else {
                    ROS_WARN("KCL: (%s) There were syntax errors in the problem file.", ros::this_node::getName().c_str());
                }
            }
        }
    }

} // close namespace
