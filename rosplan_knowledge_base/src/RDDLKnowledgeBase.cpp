
#include <rosplan_knowledge_base/RDDLKnowledgeBase.h>
#include <rosplan_knowledge_base/RDDLExprUtils.h>
#include <rosplan_knowledge_base/RDDLUtils.h>

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
        for (auto it = domain_parser.rddlTask->types.begin(); it != domain_parser.rddlTask->types.end(); ++it) {
            // FIXME should check if a type is a already supertype so we have disjunct sets?
            // FIXME filter out basic types?
            if (it->first == "int" or it->first == "real" or it->first == "bool" or it->first == "object") continue;
            types.insert(it->first);
            if (it->second->superType) super_types.insert(it->second->superType->name);
        }

        res.types.assign(types.begin(), types.end());
        res.super_types.assign(super_types.begin(), super_types.end());
        return true;
    }


    bool RDDLKnowledgeBase::getEnumTypes(rosplan_knowledge_msgs::GetEnumerableTypeService::Request &req,
                      rosplan_knowledge_msgs::GetEnumerableTypeService::Response &res) {
        for (auto it = domain_parser.rddlTask->types.begin(); it != domain_parser.rddlTask->types.end(); ++it) {
            if (it->first == req.type_name) {
                if (it->second->objects.size() > 0 and it->second->objects[0]->name[0] == '@') {
                    for (auto oit = it->second->objects.begin(); oit != it->second->objects.end(); ++oit)
                    res.values.push_back((*oit)->name);
                }
                else ROS_ERROR("KCL: (%s) Type \"%s\" is not an enumerable.", ros::this_node::getName().c_str(), req.type_name.c_str());
                break;
            }
        }
        return true;
    }

    /* get domain predicates */
    bool RDDLKnowledgeBase::getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request &req,
                                          rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {
        // FIXME I consider a predicate any state-fluent or non-fluent with a bool type
        // FIXME what about interm fluents and observ-fluents? Not sure if supported by the parser yet
        for (auto it = domain_parser.rddlTask->variableDefinitions.begin();
             it != domain_parser.rddlTask->variableDefinitions.end(); ++it) {

            // Check if the parametrized variables (pVariables) are predicates
            if ((it->second->variableType != ParametrizedVariable::STATE_FLUENT and
                 it->second->variableType != ParametrizedVariable::NON_FLUENT) or
                it->second->valueType->name != "bool") {
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
        for (auto it = domain_parser.rddlTask->variableDefinitions.begin();
             it != domain_parser.rddlTask->variableDefinitions.end(); ++it) {

            // Check if the parametrized variables (pVariables) are functions
            if ((it->second->variableType != ParametrizedVariable::STATE_FLUENT and
                 it->second->variableType != ParametrizedVariable::NON_FLUENT) or
                (it->second->valueType->name ==
                 "bool") // Skip only boolean, as all the other fluents will be handled as functions
                /*(it->second->valueType->name != "int" and  it->second->valueType->name != "real")*/)
                continue; // Skip

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
        for (auto it = domain_parser.rddlTask->variableDefinitions.begin();
             it != domain_parser.rddlTask->variableDefinitions.end(); ++it) {
            if (it->second->variableType != ParametrizedVariable::ACTION_FLUENT) continue;

            // Operator name
            rosplan_knowledge_msgs::DomainFormula formula;
            formula.name = it->second->variableName;

            // Operator parameters
            formula.typed_parameters = getTypedParams(it->second->params);

            res.operators.push_back(formula);
        }

        // Add exogenous operator
        rosplan_knowledge_msgs::DomainFormula formula;
        formula.name = "exogenous";
        // FIXME parameters?
        res.operators.push_back(formula);

        return true;
    }

    /* get domain operator details */
    bool RDDLKnowledgeBase::getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request &req,
                                               rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) {
        // FIXME maybe could precompute them?
        for (auto it = domain_parser.rddlTask->variableDefinitions.begin();
             it != domain_parser.rddlTask->variableDefinitions.end(); ++it) {
            if ((it->second->variableType != ParametrizedVariable::ACTION_FLUENT) or
                (it->second->variableName != req.name))
                continue;
            // operator name
            res.op.formula.name = it->second->variableName;

            // operator parameters
            res.op.formula.typed_parameters = getTypedParams(it->second->params);

            // Compute preconditions
            PosNegDomainFormula prec = RDDLUtils::getOperatorPreconditions(res.op.formula,
                                                                           domain_parser.uninstantiated_SACs);
            res.op.at_start_simple_condition = prec.pos;
            res.op.at_start_neg_condition = prec.neg;

            // Compute effects
            EffectDomainFormula eff = RDDLUtils::getOperatorEffects(res.op.formula,
                                                                    domain_parser.rddlTask->CPFDefinitions);
            res.op.at_end_add_effects = eff.add;
            res.op.at_end_del_effects = eff.del;
            res.op.probabilistic_effects = eff.prob;

            // Compute assign effects
            res.op.at_end_assign_effects = RDDLUtils::getOperatorAssignEffects(res.op.formula,
                                                                               domain_parser.rddlTask->CPFDefinitions);
            return true;
        }
        if (req.name == "exogenous") {
            // operator name
            res.op.formula.name = req.name;

            // Compute effects
            EffectDomainFormula eff = RDDLUtils::getOperatorEffects(res.op.formula,
                                                                    domain_parser.rddlTask->CPFDefinitions);
            res.op.at_end_add_effects = eff.add;
            res.op.at_end_del_effects = eff.del;
            res.op.probabilistic_effects = eff.prob;
            return true;
        }
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

            // predicate is sensed
            if (sensed_predicates.find(it->second->variableName) == sensed_predicates.end()) {
                sensed_predicates[it->second->variableName] = false;
            }
            res.is_sensed = sensed_predicates[it->second->variableName];


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
        loadFactsAndFunctions(); // Sets model_facts and model_functions
        // model_goals will not be filled as there are no goals in RDDL
        // model_timed_initial_literals will not be filled either
        loadMetric(); // model_metric;

        // See if there is a goal defined
        //model_goals = RDDLUtils::getGoals(domain_parser.rddlTask->CPFDefinitions);

        _horizon = domain_parser.rddlTask->horizon;
        _discount_factor = domain_parser.rddlTask->discountFactor;
        _max_nondef_actions = domain_parser.rddlTask->numberOfConcurrentActions;

        // FIXME no goals defined in RDDL -> define way of specifying goals?s
        // FIXME no timed_initial_literals defined in RDDL
        // FIXME oneof constraints?
    }

    void RDDLKnowledgeBase::addConstants() {
        // FIXME model_constants?
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Private methods

    /* Get ROS-like typed parameters */
    std::vector<diagnostic_msgs::KeyValue> RDDLKnowledgeBase::getTypedParams(const std::vector<Parameter *> &params) {
        std::vector<diagnostic_msgs::KeyValue> ros_params;

        // variabledefinitions doesn't have param names but only the type, so we're naming them here
        // Naming convention will be type initial, type initial 1, ...
        std::map<std::string, std::vector<std::string>> param_names; // key = type name, values = list of param names of that type

        for (auto pit = params.begin(); pit != params.end(); ++pit) {
            diagnostic_msgs::KeyValue param;
            param.key = (*pit)->name; // Parameter name
            size_t pos = param.key.find('?');
            if (pos != std::string::npos) param.key.erase(pos, 1); // Remove the ? if present

            param.value = (*pit)->type->name; // Type name

            // Check name
            if (param.key == param.value) { // Pram name equals type
                std::string pname(1, param.key[0]);
                auto pname_it = param_names.find(param.value);
                if (pname_it != param_names.end()) pname += std::to_string(pname_it->second.size());
                param_names[param.value].push_back(pname);
                param.key = pname;
            }

            ros_params.push_back(param);
        }
        return ros_params;
    }

    /* Get object instances */
    std::map<std::string, std::vector<std::string> >
    RDDLKnowledgeBase::getInstances(const std::map<std::string, Object *> &instance_objects) {
        map<string, std::vector<string>> instances;
        for (auto it = instance_objects.begin(); it != instance_objects.end(); ++it) {
            if (it->first == "false" or it->first == "true") continue; // Skip the "true" and "false" objects
            instances[it->second->type->name].push_back(it->first);
        }
        return instances;
    }


    /* Get facts from the task */
    void RDDLKnowledgeBase::loadFactsAndFunctions() {
        // Map to store the items with the key its grounded variable name. Initialized with the default values,
        // they will be overwritten with the initial state information
        std::map<std::string, rosplan_knowledge_msgs::KnowledgeItem> factsfuncs;

        // FIXME is this right?
        // Add all the predicates/functions from pvariables whose initial value is true or are numeric and have default value
        for (auto it = domain_parser.rddlTask->variableDefinitions.begin();
             it != domain_parser.rddlTask->variableDefinitions.end(); ++it) {
            // Check if the parametrized variables (pVariables) are predicates
            bool isBool = it->second->valueType->name == "bool";
            if ((it->second->variableType != ParametrizedVariable::STATE_FLUENT and
                 it->second->variableType != ParametrizedVariable::NON_FLUENT) or
                (isBool and it->second->initialValue == 0)) { // If it is bool but default to false ignore it
                continue; // Skip
            }

            rosplan_knowledge_msgs::KnowledgeItem item; // placeholder for the recursive function
            fillKIAddAllGroundedParameters(it->second, factsfuncs, item); // It adds them to facts
        }

        // Get state fluents from the initial state
        for (auto it = domain_parser.rddlTask->stateFluents.begin();
             it != domain_parser.rddlTask->stateFluents.end(); ++it) {
            factsfuncs[(*it)->fullName] = fillKI(*it, (*it)->params,
                                                 (*it)->initialValue); // This will override any default variable set
        }

        // Get non-fluents from the initial state
        for (auto it = domain_parser.rddlTask->nonFluents.begin();
             it != domain_parser.rddlTask->nonFluents.end(); ++it) {
            factsfuncs[(*it)->fullName] = fillKI(*it, (*it)->params,
                                                 (*it)->initialValue); // This will override any default variable set
        }

        // Iterate map to fill the right structures
        for (auto it = factsfuncs.begin(); it != factsfuncs.end(); ++it) {
            if (it->second.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT)
                model_facts.push_back(it->second);
            else if (it->second.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
                model_functions.push_back(it->second);
        }

    }

    rosplan_knowledge_msgs::KnowledgeItem
    RDDLKnowledgeBase::fillKI(const ParametrizedVariable *var, const std::vector<Parameter *> &params,
                              double initialValue) {
        rosplan_knowledge_msgs::KnowledgeItem item;
        item.initial_time = ros::Time::now();

        if (var->valueType->name == "bool") { // We have a fact
            item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            item.is_negative = (initialValue == 0);
        } else {
            //assert(var->valueType->name == "real" or var->valueType->name == "int");
            item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
            item.is_negative = false;
            item.function_value = initialValue;
        }
        item.attribute_name = var->variableName;
        item.instance_type = var->valueType->name; // Add type

        for (size_t i = 0; i < params.size(); ++i) {
            diagnostic_msgs::KeyValue param;
            param.key = domain_parser.rddlTask->variableDefinitions[var->variableName]->params[i]->name; // Get the variable name/id from the variabledefinitions structure
            size_t pos = param.key.find('?');
            if (pos != std::string::npos) param.key.erase(pos, 1); // Remove the ? if present
            param.value = params[i]->name;
            item.values.push_back(param);
        }

        return item;
    }

    void RDDLKnowledgeBase::fillKIAddAllGroundedParameters(const ParametrizedVariable *var,
                                                           std::map<std::string, rosplan_knowledge_msgs::KnowledgeItem> &factsfuncs,
                                                           rosplan_knowledge_msgs::KnowledgeItem &item,
                                                           std::string ground_params, int param_index) {
        if (param_index == 0) { // Initialize parameter
            item = rosplan_knowledge_msgs::KnowledgeItem();
            item.initial_time = ros::Time::now();

            if (var->valueType->name == "bool") { // We have a fact
                item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
                item.is_negative = (var->initialValue == 0);
            } else {
                //assert(var->valueType->name == "real" or var->valueType->name == "int");
                item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
                item.is_negative = false;
                item.function_value = var->initialValue;
            }

            item.attribute_name = var->variableName;
            item.instance_type = var->valueType->name;
            item.values.resize(var->params.size());
        }

        if (item.values.size() > 0) { // Do we have to process the parameters?
            // Set variable type for param_index
            item.values[param_index].key = domain_parser.rddlTask->variableDefinitions[var->variableName]->params[param_index]->name; // Get the variable name/id from the variabledefinitions structure
            size_t pos = item.values[param_index].key.find('?');
            if (pos != std::string::npos) item.values[param_index].key.erase(pos, 1); // Remove the ? if present

            // Instanciate the param param_index with all the instances
            const std::vector<std::string> &instance_names = model_instances[var->params[param_index]->type->name];
            for (auto it = instance_names.begin(); it != instance_names.end(); ++it) {
                item.values[param_index].value = *it;

                std::string aux = (ground_params.size()) ? ground_params + ", " + *it
                                                         : *it; // If it's first, don't add comma
                if (param_index == var->params.size() - 1) {
                    factsfuncs[var->variableName + "(" + aux +
                               ")"] = item; // If grounded all params, save it (push_back copies the object)
                } else
                    fillKIAddAllGroundedParameters(var, factsfuncs, item, aux,
                                                   param_index + 1); // If still params to ground ground them all
            }
        } else factsfuncs[var->variableName] = item; // Add the item without parameters

    }

    void RDDLKnowledgeBase::loadMetric() {
        model_metric.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::EXPRESSION;
        model_metric.expr = RDDLExprUtils::getExpression(domain_parser.rddlTask->rewardCPF);
        model_metric.optimization = "maximize"; // In RDDL we are always maximizing reward
    }

    void RDDLKnowledgeBase::parseDomain(const std::string &domain_file_path, const std::string &problem_file_path) {
        domain_path_ = domain_file_path;
        RDDLTask *t = domain_parser.parseTask(domain_file_path, problem_file_path); // The parser stores the task
        if (t == nullptr) {
            ROS_ERROR("KCL: (%s) There were syntax errors in the domain or instance file.",
                      ros::this_node::getName().c_str());
            ros::shutdown();
        }
        if (problem_file_path != "") addInitialState();
    }

    RDDLKnowledgeBase::RDDLKnowledgeBase(ros::NodeHandle &n) : KnowledgeBase(n) {
        // Get default parameters
        _nh.param("horizon", _horizon, 20);
        _nh.param("discount_factor", _discount_factor, 1.0f);
        _nh.param("max_nondef_actions", _max_nondef_actions, 1);


        _getParamsService = _nh.advertiseService("state/rddl_parameters",
                                                 &KCL_rosplan::RDDLKnowledgeBase::getRDDLParams, this);
        _setRDDLDiscountFactorSrv = _nh.advertiseService("state/set_rddl_discount_factor",
                                                         &KCL_rosplan::RDDLKnowledgeBase::setRDDLDiscountFactor, this);
        _setRDDLHorizonSrv = _nh.advertiseService("state/set_rddl_horizon",
                                                  &KCL_rosplan::RDDLKnowledgeBase::setRDDLHorizon, this);
        _setRDDLMaxNonDefSrv = _nh.advertiseService("state/set_rddl_max_nondef_actions",
                                                    &KCL_rosplan::RDDLKnowledgeBase::setRDDLMAxNonDefActions, this);
        _setImmediateRewardsSrv = _nh.advertiseService("state/get_immediate_reward",
                                                       &KCL_rosplan::RDDLKnowledgeBase::computeImmediateReward, this);
        _reloadDomainStructureSrv = _nh.advertiseService("reload_rddl_domain",
                                                         &KCL_rosplan::RDDLKnowledgeBase::reloadDomain, this);
        _getEnumtypesSrv = _nh.advertiseService("domain/enumerable_type",
                                                &KCL_rosplan::RDDLKnowledgeBase::getEnumTypes, this);
        _getFluentTypeSrv = _nh.advertiseService("domain/fluent_type",
                                                 &KCL_rosplan::RDDLKnowledgeBase::getFluentType, this);
    }

    bool RDDLKnowledgeBase::getRDDLParams(rosplan_knowledge_msgs::GetRDDLParams::Request &req,
                                          rosplan_knowledge_msgs::GetRDDLParams::Response &res) {
        res.horizon = _horizon;
        res.discount_factor = _discount_factor;
        res.max_nondef_actions = _max_nondef_actions;
        return true;
    }

    bool RDDLKnowledgeBase::setRDDLDiscountFactor(rosplan_knowledge_msgs::SetFloat::Request &req,
                                                  rosplan_knowledge_msgs::SetFloat::Response &res) {
        _discount_factor = req.value;
        res.success = true;
        return true;
    }

    bool RDDLKnowledgeBase::setRDDLHorizon(rosplan_knowledge_msgs::SetInt::Request &req,
                                           rosplan_knowledge_msgs::SetInt::Response &res) {
        _horizon = req.value;
        res.success = true;
        return true;
    }

    bool RDDLKnowledgeBase::setRDDLMAxNonDefActions(rosplan_knowledge_msgs::SetInt::Request &req,
                                                    rosplan_knowledge_msgs::SetInt::Response &res) {
        _max_nondef_actions = req.value;
        res.success = true;
        return true;
    }

    bool RDDLKnowledgeBase::reloadDomain(rosplan_knowledge_msgs::ReloadRDDLDomainProblem::Request &req,
                                         rosplan_knowledge_msgs::ReloadRDDLDomainProblem::Response &res) {
        RDDLTask *t = domain_parser.parseTask(domain_path_, req.problem_path, true); // The parser stores the task
        if (t == nullptr) {
            ROS_ERROR("KCL: (%s) There were syntax errors in the domain or instance file.",
                      ros::this_node::getName().c_str());
            res.success = false;
        } else res.success = true;
        return true;
    }


    bool RDDLKnowledgeBase::computeImmediateReward(rosplan_knowledge_msgs::GetRDDLImmediateReward::Request &req, rosplan_knowledge_msgs::GetRDDLImmediateReward::Response &res) {

        // Compute current state
        State current(domain_parser.rddlTask->CPFs.size());

        for (auto it = model_facts.begin(); it != model_facts.end(); ++it) {
            // Create predicate instantiated string
            std::string predicate = it->attribute_name;
            bool params = not it->values.empty();
            if (params) predicate += "(";
            for (size_t i = 0; i < it->values.size(); ++i) {
                if (i > 0) predicate += ", ";
                predicate += it->values[i].value;
            }
            if (params) predicate += ")";

            // Check state
            auto statefluent = domain_parser.rddlTask->stateFluentMap.find(predicate);
            if (statefluent != domain_parser.rddlTask->stateFluentMap.end()) {
                int index = statefluent->second->index;
                if (index >= 0) current.state[index] = 1 - it->is_negative;
            }
        }

        for (auto it = model_functions.begin(); it != model_functions.end(); ++it) {
            // Create predicate instantiated string
            std::string predicate = it->attribute_name;
            bool params = not it->values.empty();
            if (params) predicate += "(";
            for (size_t i = 0; i < it->values.size(); ++i) {
                if (i > 0) predicate += ", ";
                predicate += it->values[i].value;
            }
            if (params) predicate += ")";

            // Check state
            auto statefluent = domain_parser.rddlTask->stateFluentMap.find(predicate);
            if (statefluent != domain_parser.rddlTask->stateFluentMap.end()) {
                int index = statefluent->second->index;
                if (index >= 0) current.state[index] = it->function_value;
            }
        }

        // Compute action state
        ActionState action_state((int)domain_parser.rddlTask->actionFluents.size());
        for (auto it = req.action.begin(); it != req.action.end(); ++it) {
            if (it->data == "noop") continue;
            // Check state
            auto actionfluent = domain_parser.rddlTask->actionFluentMap.find(it->data);
            if (actionfluent != domain_parser.rddlTask->actionFluentMap.end()) {
                int index = actionfluent->second->index;
                if (index >= 0) action_state.state[index] = 1;
            }
        }

        // Compute reward
        double reward = 0;
        domain_parser.rddlTask->rewardCPF->formula->evaluate(reward, current, action_state);
        res.reward = reward;
        return true;
    }

    void RDDLKnowledgeBase::removeFact(const rosplan_knowledge_msgs::KnowledgeItem &msg) {
        // remove domain attribute (predicate) from knowledge base
        for (auto pit = model_facts.begin(); pit!=model_facts.end(); ++pit) {
            if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
                ROS_INFO("KCL: (%s) Setting fact (%s) is_negative=%i", ros::this_node::getName().c_str(),
                         msg.attribute_name.c_str(), (1 - pit->is_negative));
                pit->is_negative = 1 - pit->is_negative; // Negate attribute
                break;
            }
        }
    }

    bool RDDLKnowledgeBase::getFluentType(rosplan_knowledge_msgs::GetRDDLFluentType::Request &req,
                                          rosplan_knowledge_msgs::GetRDDLFluentType::Response &res) {
        for (auto it = domain_parser.rddlTask->variableDefinitions.begin(); it != domain_parser.rddlTask->variableDefinitions.end(); ++it) {
            if (it->first == req.fluent_name) {
                res.type = it->second->valueType->name;
                return true;
            }
        }
        ROS_ERROR("KCL: (%s) getFluentType: Fluent %s was not found!", ros::this_node::getName().c_str(), req.fluent_name.c_str());
        return true;
    }
}
