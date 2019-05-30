//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 30/09/18.
//

#include <rosplan_knowledge_base/PPDDLKnowledgeBase.h>

namespace KCL_rosplan {
    void PPDDLKnowledgeBase::parseDomain(const std::string &domain_file_path,
                                                      const std::string &problem_file_path) {
        PPDDLDomainPtr t = domain_parser.parseDomainProblem(domain_file_path,
                                                            problem_file_path); // The parser stores the task
        if (t == nullptr) {
            ROS_ERROR("KCL: (%s) There were syntax errors in the domain or problem file.",
                      ros::this_node::getName().c_str());
            ros::shutdown();
        }
        else if (problem_file_path != "") addInitialState();
    }

    bool PPDDLKnowledgeBase::getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request &req,
                                                        rosplan_knowledge_msgs::GetDomainNameService::Response &res) {
        if (not domain_parser.domain_parsed) return false;
        res.domain_name = domain_parser.domain_name;
        return true;
    }

    std::vector<diagnostic_msgs::KeyValue>
    PPDDLKnowledgeBase::getTypedParams(const ppddl_parser::TypeList &params) {
        std::vector<diagnostic_msgs::KeyValue> ret;
        std::map<std::string, int> var_names;
        for (auto it = params.begin(); it != params.end(); ++it) {
            diagnostic_msgs::KeyValue param;
            param.value = domain_parser.domain->types().typestring(*it); // type name

            std::string vname = param.value.substr(0, 1);
            if (var_names.find(vname) == var_names.end()) var_names[vname] = 0;
            else {
                ++var_names[vname];
                vname += std::to_string(var_names[vname]);
            }

            param.key = vname;
            ret.push_back(param);
        }
        return ret;
    }

    bool PPDDLKnowledgeBase::getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request &req,
                                                   rosplan_knowledge_msgs::GetDomainTypeService::Response &res) {
        if (not domain_parser.domain_parsed) return false;
        res.types = domain_parser.domain->types().names();
        return true;
    }

    bool PPDDLKnowledgeBase::getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request &req,
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


    bool PPDDLKnowledgeBase::getPredicateDetails(
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

                // predicate is sensed
                if(sensed_predicates.find(*it) == sensed_predicates.end()) {
                    sensed_predicates[*it] = false;
                }
                res.is_sensed = sensed_predicates[*it];

                return true;
            }
        }
        return false;
    }


    bool
    PPDDLKnowledgeBase::getFunctionPredicates(
            rosplan_knowledge_msgs::GetDomainAttributeService::Request &req,
            rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {
        if (not domain_parser.domain_parsed) return false;
        const std::vector<std::string> names = domain_parser.domain->functions().names();
        for (auto it = names.begin(); it != names.end(); ++it) {
            rosplan_knowledge_msgs::DomainFormula formula;
            formula.name = *it;

            auto f = domain_parser.domain->functions().find_function(*it);
            formula.typed_parameters = getTypedParams(domain_parser.domain->functions().parameters(*f));
            res.items.push_back(formula);
        }
        return true;
    }

    bool PPDDLKnowledgeBase::getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request &req,
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
                if (var_names.find(vname) == var_names.end()) var_names[vname] = 0;
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

    bool PPDDLKnowledgeBase::getOperatorDetails(
            rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request &req,
            rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) {
        for (auto it = domain_parser.domain->actions().begin(); it != domain_parser.domain->actions().end(); ++it) {
            if (it->second->name() != req.name) continue;

            // Get formula
            std::map<ppddl_parser::Term, std::string> var_decl;
            std::map<std::string, int> var_names;
            res.op.formula.name = it->second->name();
            for (auto pit = it->second->parameters().begin(); pit != it->second->parameters().end(); ++pit) {
                diagnostic_msgs::KeyValue param;
                param.value = ppddl_parser::TypeTable::typestring(ppddl_parser::TermTable::type(*pit)); // type name

                std::string vname = param.value.substr(0, 1);
                if (var_names.find(vname) == var_names.end()) var_names[vname] = 0;
                else {
                    ++var_names[vname];
                    vname += std::to_string(var_names[vname]);
                }
                var_decl[ppddl_parser::Term(*pit)] = vname;

                param.key = vname;
                res.op.formula.typed_parameters.push_back(param);
            }

            // Preconditions
            PPDDLUtils::fillPreconditions(it->second->precondition(), domain_parser.domain, res.op.formula,
                                                  res.op.at_start_simple_condition,
                                                  res.op.at_start_neg_condition, var_decl, var_names);
            // Effects
            PPDDLUtils::fillEffects(it->second->effect(), domain_parser.domain, res.op.at_end_add_effects, res.op.at_end_del_effects,
                                            res.op.probabilistic_effects, res.op.at_end_assign_effects, var_decl);
        }
        return true;
    }

    void PPDDLKnowledgeBase::addConstants() {
        // FIXME model_constants?
    }

    void PPDDLKnowledgeBase::addInitialState() {
        // Instances
        std::map<ppddl_parser::Term, string> var_decl;
        std::vector<std::string> names = domain_parser.problem->terms().names();
        for (auto it = names.begin(); it != names.end(); ++it) {
            const ppddl_parser::Object* t = domain_parser.problem->terms().find_object(*it);
            if (t != nullptr) {
                std::string type =  domain_parser.domain->types().typestring(domain_parser.problem->terms().type(ppddl_parser::Term(*t)));
                model_instances[type].push_back(*it);
                var_decl[ppddl_parser::Term(*t)] = *it;
            }
        }

        // Facts - initial state
        for (auto it = domain_parser.problem->init_atoms().begin(); it !=  domain_parser.problem->init_atoms().end();++it) {
            rosplan_knowledge_msgs::KnowledgeItem ki;
            ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            rosplan_knowledge_msgs::DomainFormula df = PPDDLUtils::getAtom(*it, domain_parser.domain, var_decl);
            ki.attribute_name = df.name;
            ki.values = df.typed_parameters;
            model_facts.push_back(ki);
        }

        // Functions
        // init_effects are set by the parser and not usd FIXME?
        /*for (auto it = domain_parser.problem->init_effects().begin(); it != domain_parser.problem->init_effects().end(); ++it) {
            std::cout << **it << std::endl;
        }*/
        for (auto it = domain_parser.problem->init_values().begin(); it != domain_parser.problem->init_values().end(); ++it) {
            rosplan_knowledge_msgs::KnowledgeItem ki;
            ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
            ki.attribute_name =  domain_parser.domain->functions().name(it->first->function());
            ppddl_parser::TermList tl = (*it).first->terms();
            ppddl_parser::TypeList params = domain_parser.domain->functions().parameters((*it).first->function());
            assert(tl.size() == params.size());
            std::map<std::string, int> var_names;
            for (size_t i = 0; i < tl.size(); ++i) {
                diagnostic_msgs::KeyValue p;

                std::string vname = domain_parser.domain->types().typestring(params[i]).substr(0, 1);
                if (var_names.find(vname) == var_names.end()) var_names[vname] = 0;
                else {
                    ++var_names[vname];
                    vname += std::to_string(var_names[vname]);
                }

                p.key = vname;
                p.value = domain_parser.problem->terms().get_name(tl[i]); // instance name
                ki.values.push_back(p);
            }
            ki.function_value = (*it).second.double_value();
            model_functions.push_back(ki);
        }

        // Goal
        PPDDLUtils::fillGoal(domain_parser.problem->goal(), domain_parser.domain, domain_parser.problem, model_goals, var_decl, false);


        // Metric
        model_metric.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::EXPRESSION;
        model_metric.optimization = "maximize";
        const ppddl_parser::Expression* metric = &domain_parser.problem->metric();
        const ppddl_parser::Subtraction* sub = dynamic_cast<const ppddl_parser::Subtraction*>(metric);
        if (sub != nullptr) {
            // The ppddl_parser always maximizes, and represents the minimization of X as maximize (- 0 X), so we check if it's a 0-X case
            const ppddl_parser::Value* op1 = dynamic_cast<const ppddl_parser::Value*>(&sub->operand1());
            if (op1 != nullptr && op1->value().double_value()==0) {
                // It is a 0-X case, so the metric is minimizing, and the value is the operand2.
                model_metric.optimization = "minimize";
                metric = &sub->operand2();
            }
        }
        model_metric.expr = PPDDLUtils::getExpression(*metric, domain_parser.domain, var_decl);

    }
}
