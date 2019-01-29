/*
 * Copyright [2019] <KCL - ISR collaboration>
 *
 * KCL: King's College London
 * ISR: Institue for Systems and Robotics
 *
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk), Oscar Lima (olima_84@yahoo.com)
 *
 * Helper object to simulate actions. Query KB for action effects so later those can be applied to
 * KB and update actions (simulate actions).
 *
 */

/** @file */

#include "rosplan_action_interface/ActionSimulator.h"

ActionSimulator::ActionSimulator() : nh_("~") {}

ActionSimulator::ActionSimulator(bool mirror_KB_at_startup, bool mirror_facts_and_goals) : nh_("~")
{
    // query param server for kb name and setup services to get real KB information
    prepareServices();

    if(mirror_KB_at_startup)
        // call real KB to get all of its data, save in member variables
        mirrorKB(mirror_facts_and_goals);
}

ActionSimulator::~ActionSimulator() {}

void ActionSimulator::init()
{
    // prepare services to connect to real KB and get domain data

    // query param server for kb name and setup services to get real KB information
    prepareServices();

    // call real KB to get all of its data, save in member variables, fetch predicates and goals as well
    mirrorKB(true);
}

void ActionSimulator::prepareServices()
{
    // get KB name from param server
    std::string kb;
    nh_.param<std::string>("knowledge_base", kb, "rosplan_knowledge_base");

    // prepare service to get domain operator details (action effect and preconditions)
    std::stringstream ss;
    ss << "/" << kb << "/domain/operator_details";
    od_srv_client_ = nh_.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str());

    // prepare service to get all domain operator names (all actions)
    ss.str("");
    ss << "/" << kb << "/domain/operators";
    on_srv_client_ = nh_.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>(ss.str());

    // prepare service to get all domain predicates
    ss.str("");
    ss << "/" << kb << "/domain/predicates";
    dp_srv_client_ = nh_.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(ss.str());

    // prepare service to get all state propositions
    ss.str("");
    ss << "/" << kb << "/state/propositions";
    sp_srv_client_ = nh_.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());

    // prepare service to get all state goals
    ss.str("");
    ss << "/" << kb << "/state/goals";
    sg_srv_client_ = nh_.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());
}

bool ActionSimulator::mirrorKB(bool mirror_facts_and_goals)
{
    // call real KB services and save them in member variables
    // this includes all operator names, operator details, etc. This funcion should only be called once
    // and is called automatically from constructor

    // get a list of all operator names, save in operator_names_
    if(!getOperatorNames(operator_names_))
        return false;

    // get a list of all domain predicates, save in domain_predicates_
    if(!getAllPredicateNames(domain_predicates_))
        return false;

    // get a list of all domain operator details, save in domain_operator_details_
    if(!getAllOperatorDetails(domain_operator_details_))
        return false;

    // make map between operator_names_ and domain_operator_details_, save in domain_operator_map_
    // make sure inputs are not empty
    if(!operator_names_.size() > 0 && !domain_operator_details_.size() > 0) {
        ROS_ERROR("operator_names_ or domain_operator_details_ is empty");
        return false;
    }
    // iterator over operator_names_
    domain_operator_map_.clear(); // delete previous data if any
    auto doit = domain_operator_details_.begin();
    for(auto it=operator_names_.begin(); it!=operator_names_.end(); it++) {
        domain_operator_map_.insert(std::pair<std::string, rosplan_knowledge_msgs::DomainOperator>(*it, *doit++));
    }

    if(mirror_facts_and_goals) {

        // get all state grounded predicates, save them in kb_facts_
        if(!getAllGroundedFacts(kb_facts_))
            return false;

        // get all state goals from real KB, save them in kb_goals_
        if(!getAllGoals(kb_goals_))
            return false;
    }

    return true;
}

bool ActionSimulator::checkServiceExistance(ros::ServiceClient &srv)
{
    // check srv existance
    if(!srv.waitForExistence(ros::Duration(10)))
    {
        ROS_ERROR("KCL: (ActionSimulator) Service %s not found", srv.getService().c_str());
        return false;
    }

    return true;
}

bool ActionSimulator::getOperatorNames(std::vector<std::string> &operator_names)
{
    // returns by reference a list of domain operator names

    // wait for service
    if(!checkServiceExistance(on_srv_client_))
        return false;

    // call srv
    rosplan_knowledge_msgs::GetDomainOperatorService srv;

    if(on_srv_client_.call(srv))
    {
        // clear previous data, if any
        operator_names.clear();

        // iterate over domain operator names
        for(auto it=srv.response.operators.begin(); it!= srv.response.operators.end(); it++) {
            // return by reference a list of domain operator names
            operator_names.push_back(it->name);
        }
    }
    else {
        ROS_ERROR("KCL: (ActionSimulator) Could not call Knowledge Base service: %s", on_srv_client_.getService().c_str());
        return false;
    }

    return true;
}

bool ActionSimulator::getOperatorDetails(std::string &operator_name, rosplan_knowledge_msgs::DomainOperator &op)
{
    // get from KB a single domain operator detail, based on input name (operator_name)
    // the value gets written by reference and should be passed in op

    // wait for service
    if(!checkServiceExistance(od_srv_client_))
        return false;

    rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
    srv.request.name = operator_name; // pddl_action_name

    if(od_srv_client_.call(srv)){
        // write return value by reference
        op = srv.response.op;
    }
    else {
        ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for operator details, %s", od_srv_client_.getService().c_str());
        return false;
    }

    return true;
}

bool ActionSimulator::getAllOperatorDetails(std::vector<rosplan_knowledge_msgs::DomainOperator> &domain_operator_details)
{
    // fetch some domain operator details (depending on operator_names) and store them locally for future use

    // check if we have stored operator names before
    std::vector<std::string> operator_names;
    if(operator_names_.size() > 0)
        // use stored operator names to save one service call
        operator_names = operator_names_;
    else
        // get all operator names, store them in operator_names
        getOperatorNames(operator_names);

    // make sure input is not empty
    if(!operator_names.size() > 0) {
        ROS_ERROR("operator_names is empty, while trying to get all operator details");
        return false;
    }

    // delete previous data, if any
    domain_operator_details.clear();

    // iterate over domain operator names and get their details
    for(auto it=operator_names.begin(); it != operator_names.end(); it++)
    {
        rosplan_knowledge_msgs::DomainOperator op;

        if(!getOperatorDetails(*it, op))
            return false;

        // call service to get domain operator details
        domain_operator_details.push_back(op);
    }

    return true;
}

bool ActionSimulator::getPredicatesDetails(std::vector<rosplan_knowledge_msgs::DomainFormula> &domain_predicate_details)
{
    // get all domain predicate details

    // wait for service: /rosplan_knowledge_base/domain/predicates
    if(!checkServiceExistance(dp_srv_client_))
        return false;

    // call srv
    rosplan_knowledge_msgs::GetDomainAttributeService srv;

    if(dp_srv_client_.call(srv))
    {
        // clear previous data, if any
        domain_predicate_details.clear();

        for(auto it=srv.response.items.begin(); it!= srv.response.items.end(); it++) {
            // return by reference a list of domain operator names
            domain_predicate_details.push_back(*it);
        }
    }
    else {
        ROS_ERROR("KCL: (ActionSimulator) Could not call Knowledge Base service: %s", dp_srv_client_.getService().c_str());
        return false;
    }

    return true;
}

bool ActionSimulator::getAllPredicateNames(std::vector<std::string> &domain_predicates)
{
    // get all domain predicates details, extract their names, return them by reference

    // call KB to get all predicate details
    std::vector<rosplan_knowledge_msgs::DomainFormula> domain_predicate_details;
    if(!getPredicatesDetails(domain_predicate_details))
        return false;

    //iterate over domain_predicate_details, get only names out of it and store in member variable
    for(auto it=domain_predicate_details.begin(); it!=domain_predicate_details.end(); it++) {
        domain_predicates.push_back(it->name);
    }

    return true;
}

bool ActionSimulator::getAllKnowledgeItems(ros::ServiceClient &srv_client,
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> &knowledge)
{
    // function either to get all predicates or all goals, depending on the input client

    // wait for service
    if(!checkServiceExistance(srv_client))
        return false;

    // call srv
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = ""; // get all knowledge (all facts or goals)

    if(srv_client.call(srv))
    {
        // clear previous data, if any
        knowledge.clear();

        for(auto it=srv.response.attributes.begin(); it!= srv.response.attributes.end(); it++) {
            // return by reference a list of domain operator names
            knowledge.push_back(*it);
        }
    }
    else {
        ROS_ERROR("KCL: (ActionSimulator) Could not call Knowledge Base service: %s", srv_client.getService().c_str());
        return false;
    }

    return true;
}

bool ActionSimulator::getAllGroundedFacts(std::vector<rosplan_knowledge_msgs::KnowledgeItem> &all_facts)
{
    // get all grounded facts in the KB
    return getAllKnowledgeItems(sp_srv_client_, all_facts);
}

bool ActionSimulator::saveKBSnapshot()
{
    // fetch goals and facts from the real KB, save them in internal KB

    // delete previous data if any
    kb_goals_.clear();
    kb_facts_.clear();

    // get all state grounded predicates, save them in kb_facts_
    if(!getAllGroundedFacts(kb_facts_))
        return false;

    // get all state goals from real KB, save them in kb_goals_
    if(!getAllGoals(kb_goals_))
        return false;

    return true;
}

std::string ActionSimulator::convertPredToString(std::string &predicate_name, std::vector<std::string> &params)
{
    // handy function to help facilitate the printing of predicates

    std::stringstream predicate_as_string;

    predicate_as_string << predicate_name;

    bool empty_params = false;
    if(params.size() > 0) {
        if(params.at(0) == "")
            empty_params = true;
    }

    if(!empty_params)
        for(auto it=params.begin(); it!=params.end(); it++) {
            predicate_as_string << " ";
            predicate_as_string << *it;
        }

    return predicate_as_string.str();
}

std::string ActionSimulator::convertPredToString(rosplan_knowledge_msgs::KnowledgeItem &knowledge_item)
{
    std::stringstream predicate_ss;

    predicate_ss << knowledge_item.attribute_name;

    for(auto vit=knowledge_item.values.begin(); vit!=knowledge_item.values.end(); vit++) {
        predicate_ss << " ";
        predicate_ss << vit->value;
    }

    return predicate_ss.str();
}

void ActionSimulator::printArrayKI(std::vector<rosplan_knowledge_msgs::KnowledgeItem> ki_array,
    std::string header_msg)
{
    // print all elements in a KnowledgeItem array (used for printing all facts or goals)

    // print header msg
    ROS_INFO("%s", header_msg.c_str());

    // handle empty array scenario
    if(!ki_array.size() > 0) {
        ROS_INFO("empty!");
        ROS_INFO("...................");
        return;
    }

    // non empty array scenario
    for(auto it=ki_array.begin(); it!=ki_array.end(); it++)
    {
        if(it->is_negative) {
            ROS_INFO("(not(%s))", convertPredToString(*it).c_str());
        }
        else {
            ROS_INFO("(%s)", convertPredToString(*it).c_str());
        }
    }

    ROS_INFO("...................");
}

void ActionSimulator::printInternalKBFacts()
{
    // print all predicates in internal KB
    printArrayKI(kb_facts_, std::string("....internal KB Facts....."));
}

void ActionSimulator::printInternalKBGoals()
{
    // print all predicates in internal KB
    printArrayKI(kb_goals_, std::string("....internal KB Goals....."));
}

void ActionSimulator::backupInternalKB()
{
    // save internal KB in second member variable for reset purposes: in case user wants to go
    // back to previoud KB version, without having to perform another service call
    kb_facts_bkp_ = kb_facts_;
}

bool ActionSimulator::findKnowledgeItem(std::string &predicate_name, std::vector<std::string> &args,
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator &kiit,
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> &item_array)
{
    // find predicate (fact or goal) inside knowledge item array, return by reference an iterator to the element

    // check that internal KB has at least 1 element
    if(!item_array.size() > 0) {
        ROS_ERROR("item_array is empty");
        return false;
    }

    // iterate over internal KB
    for(auto it=item_array.begin(); it!=item_array.end(); it++)
    {
        // ensure we are dealing with a fact
        if(!it->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT)
            continue;

        // discriminate by predicate name
        if(!(it->attribute_name == predicate_name))
            continue;

        // account for predicates with no arguments
        bool no_args = false;
        if(args.size() == 0)
            no_args = true;
        if(args.size() == 1)
            if(args.at(0) == "")
                no_args = true;
        if(no_args)
        {
            // check internal KB item knowledge arg size
            if(it->values.size() == 0) {
                // found element (no args), return iterator "it"
                kiit = it;
                // in theory there should'nt be duplicate predicates in KB, so no need to look for more
                return true;
            }
            else
                continue;
        }

        // compare arguments one by one
        // first ensure they have the same amount of parameters
        // they should, because they have the same predicate name...
        if(args.size() != it->values.size()) {
            ROS_ERROR("found predicates with same name but different arg size, while finding predicate.");
            ROS_ERROR("your query : (%s) vs KB : (%s)", convertPredToString(predicate_name, args).c_str(), convertPredToString(*it).c_str());
            return false;
        }
        bool predicate_have_same_args = true;
        // start comparing args one by one, return false at the first arg that is not same
        for(auto ait=args.begin(); ait!=args.end(); ait++) {
            if(*ait != it->values.at(ait - args.begin()).value) {
                predicate_have_same_args = false; // flag to jump in the outside for loop
                break; // break this inner for loop, not the outer one
            }
        }
        if(!predicate_have_same_args)
            continue;

        // found element (has args), return iterator "it"
        kiit = it;

        // in theory there should'nt be duplicate predicates in KB, so no need to look for more
        return true;
    }

    return false;
}

bool ActionSimulator::findPredicateInternal(rosplan_knowledge_msgs::KnowledgeItem &predicate, bool is_fact)
{
    // find either a predicate (goal or fact) with KnowledgeItem input

    // extract params from KnowledgeItem
    std::vector<std::string> params;
    for(auto it=predicate.values.begin(); it!=predicate.values.end(); it++)
        params.push_back(it->value);

    if(is_fact)
        // fact, call fact search function
        return findFactInternal(predicate.attribute_name, params);
    else
        // goal, call goal search function
        return findGoalInternal(predicate.attribute_name, params);
}

bool ActionSimulator::findFactInternal(std::string &predicate_name, std::vector<std::string> &args,
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator &kiit)
{
    return findKnowledgeItem(predicate_name, args, kiit, kb_facts_);
}

bool ActionSimulator::findFactInternal(std::string &predicate_name, std::vector<std::string> args)
{
    // overloaded method offered to call findFactInternal() when we don't care about the returned iterator
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kiit;
    return findFactInternal(predicate_name, args, kiit);
}

bool ActionSimulator::findFactInternal(std::string &predicate_name)
{
    // overloaded method offered to call findFactInternal() when we don't care about the returned iterator
    // and args are empty
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kiit;
    std::vector<std::string> args = {""};
    return findFactInternal(predicate_name, args, kiit);
}

bool ActionSimulator::findFactInternal(std::vector<std::string> predicate_name_and_params)
{
    // overloaded method offered to call findFactInternal() when we don't care about the returned iterator
    // and args are empty, but we have a single vector in which the first element is the predicate name

    if(!predicate_name_and_params.size() > 0) {
        ROS_ERROR("cannot find fact, received empty input");
        return false;
    }

    // create dummy empty iterator
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kiit;

    // iterate over args, skipping first element
    std::vector<std::string> args;
    for(auto it=std::next(predicate_name_and_params.begin()); it!=predicate_name_and_params.end(); it++) {
        args.push_back(*it);
    }

    return findFactInternal(predicate_name_and_params.at(0), args, kiit);
}

bool ActionSimulator::findFactInternal(rosplan_knowledge_msgs::KnowledgeItem &fact)
{
    // overloaded method that allows to find a single fact in KB with an input KnowledgeItem
    return findPredicateInternal(fact, true);
}

bool ActionSimulator::findGoalInternal(std::string &predicate_name, std::vector<std::string> &args,
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator &kiit)
{
    return findKnowledgeItem(predicate_name, args, kiit, kb_goals_);
}

bool ActionSimulator::findGoalInternal(std::string &predicate_name, std::vector<std::string> &args)
{
    // overloaded method offered to call findFactInternal() when we don't care about the returned iterator
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kiit;
    return findGoalInternal(predicate_name, args, kiit);
}

bool ActionSimulator::findGoalInternal(rosplan_knowledge_msgs::KnowledgeItem &goal)
{
    // overloaded method that allows to find a single goal in KB with an input KnowledgeItem
    return findPredicateInternal(goal, false);
}

bool ActionSimulator::removePredicateInternal(std::string &predicate_name, std::vector<std::string> &args,
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> &item_array)
{
    // remove predicate from KnowledgeItem array

    // find element in KB and get a pointer to it
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator it;

    if(findFactInternal(predicate_name, args, it)) {
        // element exists, delete knowledge item from internal KB
        item_array.erase(it);
    }
    else {
        // element does not exist
        ROS_ERROR("Could not find predicate in internal KB (%s), while trying to delete",
                  convertPredToString(predicate_name, args).c_str());
        return false;
    }

    return true;
}

bool ActionSimulator::removeFactInternal(std::string &predicate_name, std::vector<std::string> args)
{
    // remove fact from internal KB
    return removePredicateInternal(predicate_name, args, kb_facts_);
}

bool ActionSimulator::removeFactInternal(std::vector<std::string> &predicate_and_args)
{
    // remove fact from internal KB, this in an overloaded method

    // make sure input is not empty
    if(!predicate_and_args.size() > 0)
        return false;

    // prepare args to be able to call removeFactInternal() with predicate name and args separately
    std::string predicate_name = predicate_and_args.at(0);

    if(predicate_and_args.size() == 1) {
        // no args
        std::vector<std::string> empty_args = {""};
        return removeFactInternal(predicate_name, empty_args);
    }

    std::vector<std::string> args;
    // std::next -> iterate starting from second element
    for(auto it=std::next(predicate_and_args.begin()); it!=predicate_and_args.end(); it++) {
        args.push_back(*it);
    }

    return removeFactInternal(predicate_name, args);
}

bool ActionSimulator::removeFactInternal(rosplan_knowledge_msgs::DomainFormula &predicate)
{
    // overloaded method that allows to remove facts with DomainFormula input format

    // extract args from predicate
    std::vector<std::string> params;
    for(auto it=predicate.typed_parameters.begin(); it!=predicate.typed_parameters.end(); it++) {
        params.push_back(it->value);
    }

    // call original method with adjusted args
    return removeFactInternal(predicate.name, params);
}

bool ActionSimulator::removeGoalInternal(std::string &predicate_name, std::vector<std::string> &args)
{
    // remove fact from internal KB
    return removePredicateInternal(predicate_name, args, kb_goals_);
}

void ActionSimulator::addFactInternal(std::string &predicate_name, std::vector<std::string> params)
{
    // add fact to internal KB

    // make sure fact is not already there
    if(findFactInternal(predicate_name, params)) {
        // element exists
        ROS_WARN("Tried to add fact twice, doing nothing... (%s)", convertPredToString(predicate_name, params).c_str());
        return;
    }

    // kb_facts_ is of type -> std::vector<rosplan_knowledge_msgs::KnowledgeItem>

    rosplan_knowledge_msgs::KnowledgeItem knowledge_item;

    knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
    knowledge_item.is_negative = false; // negative predicates not supported (close world assumption)
    knowledge_item.attribute_name = predicate_name;

    bool is_params_empty = false;
    if(params.size() == 0)
        is_params_empty = true;
    if(params.size() == 1) {
        if(params.at(0) == "")
            is_params_empty = true;
    }

    if(!is_params_empty)
    {
        // non empty args

        // iterate over params
        for(auto it=params.begin(); it!=params.end(); it++) {
            diagnostic_msgs::KeyValue params_key_value;
            params_key_value.key = "";
            params_key_value.value = *it;
            knowledge_item.values.push_back(params_key_value);
        }
    }

    // add knowledge item to internal KB
    kb_facts_.push_back(knowledge_item);
}

std::vector<std::string> ActionSimulator::groundParams(rosplan_knowledge_msgs::DomainFormula ungrounded_precondition,
        std::map<std::string, std::string> &ground_dictionary)
{
    // receive an operator and a dictionary of key values, return grounded predicate parameters

    std::vector<std::string> grounded_fact_args;

    // iterate over the ungrounded params
    for(auto it=ungrounded_precondition.typed_parameters.begin(); it!=ungrounded_precondition.typed_parameters.end(); it++) {
        std::string grounded_arg = ground_dictionary.find(it->key)->second;
        grounded_fact_args.push_back(grounded_arg);
    }

    return grounded_fact_args;
}

bool ActionSimulator::isActionAplicable(bool action_start, bool overall_preconditions, std::string &action_name, std::vector<std::string> &params,
    std::map<std::string, std::string> &ground_dictionary)
{
    // check if action start/end/overall preconditions are consistent with internal KB information
    // and return by reference the ground dictionary for simulation action purposes

    // ensure domain_operator_map_ has data
    if(!domain_operator_map_.size() > 0) {
        ROS_ERROR("domain_operator_map_ is empty");
        return false;
    }

    // get operator from action name (std::map of names, operators)
    std::map<std::string, rosplan_knowledge_msgs::DomainOperator>::iterator opit = domain_operator_map_.find(action_name);
    // check if operator was found
    if(opit == domain_operator_map_.end()) {
        ROS_ERROR("domain operator not found for action: %s", action_name.c_str());
        return false;
    }

    rosplan_knowledge_msgs::DomainOperator op = opit->second;

    // construct ground dictionary from params (key vs grounded args)
    // ensure they have the same size
    if(!(op.formula.typed_parameters.size() == params.size())) {
        ROS_ERROR("size of domain operator parameters and action params do not match, (while creating ground dictionary)");
        return false;
    }
    for(auto it=op.formula.typed_parameters.begin(); it!=op.formula.typed_parameters.end(); it++)
        // value gets passed by reference, to be used by action simulator but also is used for the rest of this function
        ground_dictionary.insert(std::pair<std::string, std::string>(it->key, params.at(std::distance(op.formula.typed_parameters.begin(), it))));

    // check action preconditions

    // check overall conditions, if neeed
    if(overall_preconditions) {
        // iterate over ungrounded overall conditions, find in KB
        for(auto it=op.over_all_simple_condition.begin(); it!=op.over_all_simple_condition.end(); it++) {
            // if not found, action is not applicable
            std::vector<std::string> gp = groundParams(*it, ground_dictionary);
            if(!findFactInternal(it->name, gp)) { // "it" is in DomainFormula format
                // inform which precondition were not met
                ROS_DEBUG("precondition not met: (%s)", convertPredToString(it->name, gp).c_str());
                return false;
            }
        }
        return true;
    }

    // check at start preconditions, if needed
    if(action_start) {
        // iterate over ungrounded positive preconditions, find in KB
        for(auto it=op.at_start_simple_condition.begin(); it!=op.at_start_simple_condition.end(); it++) {
            // if not found, action is not applicable
            std::vector<std::string> gp = groundParams(*it, ground_dictionary);
            if(!findFactInternal(it->name, gp)) { // "it" is in DomainFormula format, gp = grounded parameters
                // inform which precondition were not met
                ROS_DEBUG("at start precondition not met: (%s)", convertPredToString(it->name, gp).c_str());
                return false;
            }
        }
        // at start preconditions are met
        return true;
    }

    // check at end preconditions, if needed: this part will not be executed unless overall_preconditions and action_start are false
    // iterate over ungrounded at_start_neg_condition, make sure they are not in KB
    for(auto it=op.at_start_neg_condition.begin(); it!=op.at_start_neg_condition.end(); it++) {
        // if found, action is not applicable
        std::vector<std::string> gp = groundParams(*it, ground_dictionary);
        if(findFactInternal(it->name, gp)) { // "it" is in DomainFormula format
            // inform which precondition was not met
            ROS_DEBUG("at end precondition not met: (%s)", convertPredToString(it->name, gp).c_str());
            return false;
        }
    }
    // at end preconditions are met
    return true;
}

bool ActionSimulator::isActionStartAplicable(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary)
{
    // overloaded function that checks if action start preconditions are consistent with internal KB information
    // and return by reference the ground dictionary for simulation action purposes
    return isActionAplicable(true, false, action_name, params, ground_dictionary);
}

bool ActionSimulator::isActionStartAplicable(std::string &action_name, std::vector<std::string> &params)
{
    // overloaded function that checks if action start preconditions are consistent with internal KB information
    std::map<std::string, std::string> ground_dictionary;
    return isActionAplicable(true, false, action_name, params, ground_dictionary);
}

bool ActionSimulator::isActionOverAllAplicable(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary)
{
    // overloaded function that checks if action overall preconditions are consistent with internal KB information
    // and return by reference the ground dictionary for simulation action purposes
    return isActionAplicable(false, true, action_name, params, ground_dictionary);
}

bool ActionSimulator::isActionEndAplicable(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary)
{
    // overloaded function that checks if action end preconditions are consistent with internal KB information
    // and return by reference the ground dictionary for simulation action purposes
    return isActionAplicable(false, false, action_name, params, ground_dictionary);
}

bool ActionSimulator::isActionEndAplicable(std::string &action_name, std::vector<std::string> &params)
{
    // overloaded function that checks if action end preconditions are consistent with internal KB information
    std::map<std::string, std::string> ground_dictionary;
    return isActionAplicable(false, false, action_name, params, ground_dictionary);
}

bool ActionSimulator::isActionAplicable(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary)
{
    // overloaded function that checks if all action preconditions (start, end, overall) are consistent with
    // internal KB information and return by reference the ground dictionary for simulation action purposes

    if(isActionStartAplicable(action_name, params, ground_dictionary))
        // check at end conditions
        if(isActionEndAplicable(action_name, params, ground_dictionary))
            // check overall conditions
            if(isActionOverAllAplicable(action_name, params, ground_dictionary))
                return true;

    return false;
}

bool ActionSimulator::isActionAplicable(std::string &action_name, std::vector<std::string> &params)
{
    // overloaded function that checks if all action preconditions (start, end, overall) are consistent with
    // internal KB information
    std::map<std::string, std::string> ground_dictionary;
    return isActionAplicable(action_name, params, ground_dictionary);
}

bool ActionSimulator::simulateAction(std::string &action_name, std::vector<std::string> &params, bool action_start)
{
    // apply delete and add list to KB current state

    // ensure domain_operator_details_ has data
    if(!domain_operator_details_.size() > 0) {
        ROS_ERROR("domain_operator_details_ is empty");
        return false;
    }

    // check if action is applicable and get ground dictionary
    std::map<std::string, std::string> ground_dictionary;
    if(action_start) {
        // check action start preconditions
        if(!isActionStartAplicable(action_name, params, ground_dictionary)) {
            ROS_ERROR("action start (%s) is not applicable, will not simulate", action_name.c_str());
            return false;
        }
    }
    else {
        if(!isActionEndAplicable(action_name, params, ground_dictionary)) {
            ROS_ERROR("action end (%s) is not applicable, will not simulate", action_name.c_str());
            return false;
        }
    }
    // check overall action preconditions
    if(!isActionOverAllAplicable(action_name, params, ground_dictionary)) {
        ROS_ERROR("action overall (%s) is not applicable, will not simulate", action_name.c_str());
        return false;
    }

    // query action effects

    // get domain operator details corresponding to the action to simulate
    rosplan_knowledge_msgs::DomainOperator op = domain_operator_map_.find(action_name)->second;

    if(action_start)
    {
        // action start effects

        // iterate over positive start action effects and apply to KB
        for(auto it=op.at_start_add_effects.begin(); it!=op.at_start_add_effects.end(); it++)
            addFactInternal(it->name, groundParams(*it, ground_dictionary));

        // iterate over negative start action effects and apply to KB
        for(auto it=op.at_start_del_effects.begin(); it!=op.at_start_del_effects.end(); it++) {
            if(!removeFactInternal(it->name, groundParams(*it, ground_dictionary)))
                return false;
        }
    }
    else
    {
        // action end effects

        // iterate over positive start action effects and apply to KB
        for(auto it=op.at_end_add_effects.begin(); it!=op.at_end_add_effects.end(); it++)
            addFactInternal(it->name, groundParams(*it, ground_dictionary));

        // iterate over negative start action effects and apply to KB
        for(auto it=op.at_end_del_effects.begin(); it!=op.at_end_del_effects.end(); it++) {
            if(!removeFactInternal(it->name, groundParams(*it, ground_dictionary)))
                return false;
        }
    }

    return true;
}

bool ActionSimulator::simulateActionStart(std::string &action_name, std::vector<std::string> &params)
{
    // apply at start action effects to internal KB
    return simulateAction(action_name, params, true);
}

bool ActionSimulator::simulateActionEnd(std::string &action_name, std::vector<std::string> &params)
{
    // apply at end action effects to internal KB
    return simulateAction(action_name, params, false);
}

bool ActionSimulator::getAllGoals(std::vector<rosplan_knowledge_msgs::KnowledgeItem> &kb_goals)
{
    // get all goals from real KB

    // wait for service
    if(!checkServiceExistance(sg_srv_client_))
        return false;

    // call srv
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = ""; // empty means get all goals in KB

    if(sg_srv_client_.call(srv))
    {
        // clear previous data, if any
        kb_goals.clear();

        for(auto it=srv.response.attributes.begin(); it!= srv.response.attributes.end(); it++) {
            // return by reference a list of domain operator names
            kb_goals.push_back(*it);
        }
    }
    else {
        ROS_ERROR("KCL: (ActionSimulator) Could not call Knowledge Base service: %s", sg_srv_client_.getService().c_str());
        return false;
    }

    return true;
}

bool ActionSimulator::areGoalsAchieved()
{
    // check if all goals are satisfied in internal KB

    if(kb_goals_.size() == 0) {
        ROS_WARN("received empty goal query, will return true, means goals are satisfied");
        return true;
    }

    bool goal_state_reached = true;

    // iterate over kb_goals_
    for(auto it=kb_goals_.begin(); it!=kb_goals_.end(); it++) {
        // find fact in internal KB using KnowledgeItem version
        if(!findFactInternal(*it)) {
            // inform user about which goals are not achieved
            ROS_INFO("missing goal : (%s)", convertPredToString(*it).c_str());
            goal_state_reached =  false;
        }
    }

    return goal_state_reached;
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "action_simulator_tester");
    ROS_INFO("Node is going to initialize...");
    // create object of the node class (ActionSimulator)
    ActionSimulator action_simulator_tester(true, true);
    ROS_INFO("Node initialized.");

    // SNIPPETS: Example of how to use this library

    // snippet: get operator names
    std::vector<std::string> operator_names;
    action_simulator_tester.getOperatorNames(operator_names);
    ROS_INFO("get operator names:");
    ROS_INFO("===============");
    for(auto it=operator_names.begin(); it != operator_names.end(); it++)
    {
        ROS_INFO("%s", it->c_str());
    }

    // snippet: get all domain predicate names
    std::vector<std::string> domain_predicates;
    action_simulator_tester.getAllPredicateNames(domain_predicates);
    ROS_INFO("get all predicate names:");
    ROS_INFO("================");
    for(auto it=domain_predicates.begin(); it != domain_predicates.end(); it++)
    {
        ROS_INFO("%s", it->c_str());
    }

    // save all predicates to internal KB
    action_simulator_tester.saveKBSnapshot();

    // snippet: print internal KB facts
    ROS_INFO("print internal KB facts:");
    ROS_INFO("================");
    action_simulator_tester.printInternalKBFacts();

    // snippet: print internal KB goals
    ROS_INFO("print internal KB goals:");
    ROS_INFO("================");
    action_simulator_tester.printInternalKBGoals();

    // snippet: find fact in KB (with no args)
    ROS_INFO("find fact in internal KB no args:");
    ROS_INFO("================");
    std::string predicate_name2 = "person_descending";
    if(action_simulator_tester.findFactInternal(predicate_name2))
        ROS_INFO("predicate : (person_descending) found in internal KB");
    else
        ROS_INFO("predicate : (person_descending) not found in internal KB");
    // alternative option
    ROS_INFO("find fact in internal KB no args (alternative option):");
    ROS_INFO("================");
    std::string predicate_name3 = "person_descending";
    std::vector<std::string> args1 = {""};
    if(action_simulator_tester.findFactInternal(predicate_name3, args1))
        ROS_INFO("predicate : (person_descending) found in internal KB");
    else
        ROS_INFO("predicate : (person_descending) not found in internal KB");

    // snippet: find fact in KB (with args)
    ROS_INFO("find fact in internal KB with args:");
    ROS_INFO("================");
    std::string predicate_name4 = "has_driver_license";
    std::vector<std::string> args2 = {"batdad"};
    if(action_simulator_tester.findFactInternal(predicate_name4, args2))
        ROS_INFO("predicate : (has_driver_license batdad) found in internal KB");
    else
        ROS_INFO("predicate : (has_driver_license batdad) not found in internal KB");

    // snippet: remove predicate from KB (with args)
    std::string predicate_name = "has_driver_license";
    std::vector<std::string> args3 = {"batdad"};
    action_simulator_tester.removeFactInternal(predicate_name, args3);
    ROS_INFO("test: remove predicate with args (has_driver_license batdad):");
    ROS_INFO("================");
    action_simulator_tester.printInternalKBFacts();

    // snippet: remove predicate from KB (no args)
    std::string predicate_name5 = "person_descending";
    std::vector<std::string> args4 = {""};
    ROS_INFO("test: remove predicate with no args (person_descending):");
    ROS_INFO("================");
    action_simulator_tester.removeFactInternal(predicate_name5, args4);
    action_simulator_tester.printInternalKBFacts();

    // snippet: see if action is applicable
    std::string action_name = "get_down_from_car";
    std::vector<std::string> params = {"batdad","car","ben_school"}; // person, car, location
    ROS_INFO("check if action is applicable: (get_down_from_car batdad car ben_school), expected outcome is true");
    ROS_INFO("================");
    if(action_simulator_tester.isActionAplicable(action_name, params))
        ROS_INFO("action is applicable!");
    else
        ROS_INFO("action is not applicable");

    // snippet: simulate an action start
    ROS_INFO("simulate action start: (get_down_from_car batdad car ben_school)");
    ROS_INFO("================");
    ROS_INFO("KB before simulation");
    action_simulator_tester.printInternalKBFacts();
    std::string action_name2 = "get_down_from_car";
    std::vector<std::string> params2 = {"batdad","car","ben_school"};
    action_simulator_tester.simulateActionStart(action_name2, params2);
    ROS_INFO("KB after simulation");
    action_simulator_tester.printInternalKBFacts();

    // snippet: test areGoalsAchieved()
//     ROS_INFO("Check if goals are achieved:");
//     ROS_INFO("================");
//     ROS_INFO("KB goals:");
//     action_simulator_tester.printInternalKBGoals();
//     ROS_INFO("KB facts:");
//     action_simulator_tester.printInternalKBFacts();
//     if(action_simulator_tester.areGoalsAchieved())
//         ROS_INFO("goals achieved");
//     else
//         ROS_INFO("goals not achieved");

    return 0;
}
