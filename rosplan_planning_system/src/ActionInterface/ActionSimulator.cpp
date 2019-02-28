/*
 * Copyright [2019] <KCL - ISR collaboration>
 *
 * KCL: King's College London
 * ISR: Institue for Systems and Robotics
 *
 * Author: Michael Cashmore (michael.cashmore@kcl.ac.uk), Oscar Lima (olima@isr.tecnico.ulisboa.pt)
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
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> &knowledge_item_array)
{
    // function either to get all predicates or all goals, depending on the input client : srv_client

    // wait for service
    if(!checkServiceExistance(srv_client))
        return false;

    // call srv
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = ""; // get all knowledge (all facts or goals)

    if(srv_client.call(srv))
    {
        // clear previous data, if any
        knowledge_item_array.clear();

        for(auto it=srv.response.attributes.begin(); it!= srv.response.attributes.end(); it++) {
            // return by reference a list of knowledge items
            knowledge_item_array.push_back(*it);
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
    ROS_DEBUG("%s", header_msg.c_str());

    // handle empty array scenario
    if(!ki_array.size() > 0) {
        ROS_DEBUG("empty!");
        ROS_DEBUG("...................");
        return;
    }

    // non empty array scenario
    for(auto it=ki_array.begin(); it!=ki_array.end(); it++)
    {
        if(it->is_negative) {
            ROS_DEBUG("(not(%s))", convertPredToString(*it).c_str());
        }
        else {
            ROS_DEBUG("(%s)", convertPredToString(*it).c_str());
        }
    }

    ROS_DEBUG("...................");
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
        ROS_DEBUG("Could not find predicate in internal KB (%s), while trying to delete",
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

bool ActionSimulator::addFactInternal(std::string &predicate_name, std::vector<std::string> params)
{
    // add fact to internal KB

    // make sure fact is not already there
    if(findFactInternal(predicate_name, params)) {
        // element exists
        ROS_DEBUG("Tried to add fact twice, doing nothing... (%s)", convertPredToString(predicate_name, params).c_str());
        return false; // false indicates that it did nothing because it was already there
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

    return true; // true indicates that it did added the fact and that is not repeated
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

bool ActionSimulator::computeGroundDictionary(std::string &action_name, std::vector<std::string> &params,
        std::map<std::string, std::string> &ground_dictionary, rosplan_knowledge_msgs::DomainOperator &op)
{
    // get domain operator corresponding to the received action, make a map between keys and grounded action params

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

    op = opit->second;

    // construct ground dictionary from params (key vs grounded args)
    // ensure they have the same size
    if(!(op.formula.typed_parameters.size() == params.size())) {
        ROS_ERROR("size of domain operator parameters and action params do not match, (while creating ground dictionary)");
        return false;
    }
    for(auto it=op.formula.typed_parameters.begin(); it!=op.formula.typed_parameters.end(); it++)
        // value gets passed by reference, to be used by action simulator but also is used for the rest of this function
        ground_dictionary.insert(std::pair<std::string, std::string>(it->key, params.at(std::distance(op.formula.typed_parameters.begin(), it))));

    return true;
}

double ActionSimulator::getFactProbability(std::string &fact_name, std::vector<std::string> &params)
{
    // find fact in KB and get its associated probability (probability of a fact being true)

    // HACK for testing purposes, what would be the proper way is to sense predicates
    // with robot sensors, add some bayesian binary formulation where if you sense the predicate mutliple times
    // then you increase the prob of that fact being true, then upload this predicate to rosplan KB
    // but filling the probability field: rosplan_knowledge_msgs::KnowledgeItem::probability

    // remove
    ROS_DEBUG("fact name: %s", fact_name.c_str());

    if(fact_name == "machine_on")
        return 0.5;
    else if(fact_name == "machine_off")
        return 0.5;
    else if(fact_name == "robot_at")
        return 0.5;

    return 1.0;
}

bool ActionSimulator::checkConditions(bool positive_conditions, std::vector<rosplan_knowledge_msgs::DomainFormula> &df,
        std::map<std::string, std::string> &ground_dictionary, double &combined_probability)
{
    if(positive_conditions) {
        // positive preconditions

        // iterate over ungrounded positive preconditions, make sure they are found in KB
        for(auto it=df.begin(); it!=df.end(); it++) {
            // if not found, action is not applicable
            std::vector<std::string> gp = groundParams(*it, ground_dictionary);
            if(!findFactInternal(it->name, gp)) { // "it" is in DomainFormula format, gp = grounded parameters
                // inform which precondition were not met
                ROS_DEBUG("precondition not met: (%s)", convertPredToString(it->name, gp).c_str());
                return false;
            }

            // update action probability based on prob of this fact being true
            combined_probability *= getFactProbability(it->name, gp);
        }
    }
    else {
        // negative preconditions

        // iterate over ungrounded negative preconditions, make sure they are not in KB
        for(auto it=df.begin(); it!=df.end(); it++) {
            // if found, action is not applicable
            std::vector<std::string> gp = groundParams(*it, ground_dictionary);
            if(findFactInternal(it->name, gp)) { // "it" is in DomainFormula format
                // inform which precondition was not met
                ROS_DEBUG("precondition not met: (not (%s))", convertPredToString(it->name, gp).c_str());
                return false;
            }

            // update action probability based on prob of this fact being true
            combined_probability *= getFactProbability(it->name, gp);
        }
    }

    // preconditions are met
    return true;
}

bool ActionSimulator::isActionApplicable(std::string &action_name, std::vector<std::string> &params,
    bool action_start, bool action_overall, bool action_end, std::map<std::string, std::string> &ground_dictionary,
    double &combined_probability)
{
    // check if action start/end/overall preconditions are met
    // and return by reference the ground dictionary for simulation action purposes
    // and return the probability of this action to succeed, based on the combined probability of all facts being true

    rosplan_knowledge_msgs::DomainOperator op;

    // get ground dictionary, a map between keys and grounded action parameters
    if(!computeGroundDictionary(action_name, params, ground_dictionary, op)) {
        ROS_ERROR("failed to compute ground dictionary");
        return false;
    }

    // check action preconditions, compute action probability at the same time

    // initialize prob to 1.0
    combined_probability = 1.0;

    // check action start preconditions, if needed
    if(action_start) {
        ROS_DEBUG("checking action start preconditions");

        // check action start positive preconditions
        if(!checkConditions(true, op.at_start_simple_condition, ground_dictionary, combined_probability))
            return false;

        // check action start negative preconditions
        if(!checkConditions(false, op.at_start_neg_condition, ground_dictionary, combined_probability))
            return false;

        // action start preconditions are met
    }

    // check overall preconditions if needed
    if(action_overall) {
        ROS_DEBUG("checking action overall preconditions");

        // check action overall positive preconditions
        if(!checkConditions(true, op.over_all_simple_condition, ground_dictionary, combined_probability))
            return false;

        // check action overall negative preconditions
        if(!checkConditions(false, op.over_all_neg_condition, ground_dictionary, combined_probability))
            return false;

        // action overall preconditions are met
    }

    // check action end preconditions if needed
    if(action_end) {
        ROS_DEBUG("checking action end preconditions");

        // check action end positive preconditions
        if(!checkConditions(true, op.at_end_simple_condition, ground_dictionary, combined_probability))
            return false;

        // check action end negative preconditions
        if(!checkConditions(false, op.at_end_neg_condition, ground_dictionary, combined_probability))
            return false;

        // action end preconditions are met
    }

    // all requested preconditions were met
    return true;
}

bool ActionSimulator::isActionApplicable(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary)
{
    // overloaded function that checks if all action preconditions (start, end, overall) are consistent with
    // internal KB information and return by reference the ground dictionary for simulation action purposes

    double combined_probability; // dummy unused value

    // check action start, overall and end preconditions
    return isActionApplicable(action_name, params, true, true, true, ground_dictionary, combined_probability);
}

bool ActionSimulator::isActionApplicable(std::string &action_name, std::vector<std::string> &params)
{
    // overloaded function that checks if all action preconditions (start, end, overall) are consistent with
    // internal KB information
    std::map<std::string, std::string> ground_dictionary; // dummy unused value
    double combined_probability; // dummy unused value
    return isActionApplicable(action_name, params, true, true, true, ground_dictionary, combined_probability);
}

bool ActionSimulator::isActionStartApplicable(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary)
{
    // overloaded function that checks if action start and overall preconditions are met
    // and return by reference the ground dictionary for simulation action purposes
    double combined_probability; // dummy unused value
    return isActionApplicable(action_name, params, true, true, false, ground_dictionary, combined_probability);
}

bool ActionSimulator::isActionStartApplicable(std::string &action_name, std::vector<std::string> &params)
{
    // overloaded function that checks if action start preconditions are met
    std::map<std::string, std::string> ground_dictionary; // dummy unused value
    double combined_probability; // dummy unused value
    return isActionApplicable(action_name, params, true, true, false, ground_dictionary, combined_probability);
}

bool ActionSimulator::isActionStartApplicable(std::string &action_name, std::vector<std::string> &params,
        double &combined_probability)
{
    // overloaded function that checks if action start preconditions are met
    // we dont't care here about the ground dictionary but we do care about the combined probability
    std::map<std::string, std::string> ground_dictionary; // dummy unused value
    return isActionApplicable(action_name, params, true, true, false, ground_dictionary, combined_probability);
}

bool ActionSimulator::isActionEndApplicable(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary)
{
    // overloaded function that checks if action end preconditions are met
    // and return by reference the ground dictionary for simulation action purposes
    double combined_probability; // dummy unused value
    return isActionApplicable(action_name, params, false, false, true, ground_dictionary, combined_probability);
}

bool ActionSimulator::isActionEndApplicable(std::string &action_name, std::vector<std::string> &params)
{
    // overloaded function that checks if action end preconditions are met
    std::map<std::string, std::string> ground_dictionary;
    double combined_probability; // dummy unused value
    return isActionApplicable(action_name, params, false, false, true, ground_dictionary, combined_probability);
}

bool ActionSimulator::isActionEndApplicable(std::string &action_name, std::vector<std::string> &params,
        double &combined_probability)
{
    // overloaded function that checks if action end preconditions are met
    std::map<std::string, std::string> ground_dictionary; // dummy unused value
    return isActionApplicable(action_name, params, false, false, true, ground_dictionary, combined_probability);
}

rosplan_knowledge_msgs::KnowledgeItem ActionSimulator::createFactKnowledgeItem(std::string &name, std::vector<std::string> &params,
        bool is_negative)
{
    // helper function to avoid repetition of code, used to facilitate the creation of knowledge items

    rosplan_knowledge_msgs::KnowledgeItem knowledge_item;

    knowledge_item.is_negative = is_negative;
    knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
    knowledge_item.attribute_name = name;
    // transform param from std::vector<std::string> to std::vector<diagnostic_msgs::KeyValue> (diagnostic_msgs/KeyValue[] values)
    for(auto pit=params.begin(); pit!=params.end(); pit++) { // pit = parameter iterator
        diagnostic_msgs::KeyValue value;
        // key is lost! this is a weakness of storing facts as vector of strings...
        value.key = std::string("LOST");
        value.value = *pit;
        knowledge_item.values.push_back(value);
    }

    return knowledge_item;
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
        if(!isActionStartApplicable(action_name, params, ground_dictionary)) {
            ROS_ERROR("action start (%s) is not applicable, will not simulate", convertPredToString(action_name, params).c_str());
            return false;
        }
    }
    else {
        if(!isActionEndApplicable(action_name, params, ground_dictionary)) {
            ROS_ERROR("action start (%s) is not applicable, will not simulate", convertPredToString(action_name, params).c_str());
            return false;
        }
    }

    // query action effects

    // get domain operator details corresponding to the action to simulate
    rosplan_knowledge_msgs::DomainOperator op = domain_operator_map_.find(action_name)->second;

    if(action_start)
    {
        // action start effects

        // keep track of applied effects performed in KB so they can be reverted afterwards
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> a_start_effective_effects;

        // iterate over positive start action effects and apply to KB
        for(auto it=op.at_start_add_effects.begin(); it!=op.at_start_add_effects.end(); it++) {
            std::vector<std::string> params = groundParams(*it, ground_dictionary);
            // try to add fact
            if(addFactInternal(it->name, params)) {
                // fact did not existed in KB, was effectively added, therefore add to effective effects list
                a_start_effective_effects.push_back(createFactKnowledgeItem(it->name, params, false));
            }
        }

        // iterate over negative start action effects and apply to KB
        for(auto it=op.at_start_del_effects.begin(); it!=op.at_start_del_effects.end(); it++) {
            std::vector<std::string> params = groundParams(*it, ground_dictionary);
            if(removeFactInternal(it->name, params)) {
                // predicate was found and removed, add to the effective effects list
                a_start_effective_effects.push_back(createFactKnowledgeItem(it->name, params, true));
            }
        }

        // update map store in member variable the simulated action to be able to revert it
        start_sim_actions_map_[std::pair<std::string, std::vector<std::string> >(action_name, params)] = a_start_effective_effects;
    }
    else
    {
        // action end effects

        // keep track of applied effects performed in KB so they can be reverted afterwards
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> a_end_effective_effects;

        // iterate over positive start action effects and apply to KB
        for(auto it=op.at_end_add_effects.begin(); it!=op.at_end_add_effects.end(); it++) {
            std::vector<std::string> params = groundParams(*it, ground_dictionary);
            if(addFactInternal(it->name, params)) {
                // fact did not existed in KB, was effectively added, therefore add to effective effects list
                a_end_effective_effects.push_back(createFactKnowledgeItem(it->name, params, false));
            }
        }

        // iterate over negative start action effects and apply to KB
        for(auto it=op.at_end_del_effects.begin(); it!=op.at_end_del_effects.end(); it++) {
            std::vector<std::string> params = groundParams(*it, ground_dictionary);
            if(removeFactInternal(it->name, params)) {
                // predicate was found and removed, add to the effective effects list
                a_end_effective_effects.push_back(createFactKnowledgeItem(it->name, params, true));
            }
        }

        // update map store in member variable the simulated action to be able to revert it
        end_sim_actions_map_[std::pair<std::string, std::vector<std::string> >(action_name, params)] = a_end_effective_effects;
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

bool ActionSimulator::reverseEffectsFromKIA(std::vector<rosplan_knowledge_msgs::KnowledgeItem> effects)
{
    // apply effects from Knowledge item array
    // apply all of the effects in knowledge item array to a KB

    // make sure input is non empty
    if(!(effects.size() > 0)) {
        ROS_DEBUG("effects encoded as knowledge item array are empty, cannot apply empty effects");
        return false;
    }

    // iterate over effects
    for(auto eit=effects.begin(); eit!=effects.end(); eit++) {
        std::string name = eit->attribute_name;;
        std::vector<std::string> params;
        // iterate over diagnostic_msgs and get params
        for(auto pit=eit->values.begin(); pit!=eit->values.end(); pit++) {
            params.push_back(pit->value);
        }

        // query if positive or negative effect
        if(eit->is_negative) {
            // add relevant effective effect facts
            addFactInternal(name, params);
        }
        else {
            // remove relevant effective effect facts
            removeFactInternal(name, params);
        }
    }

    return true;
}

bool ActionSimulator::revertAction(std::string &action_name, std::vector<std::string> &params, bool action_start)
{
    // get from memory relevant effects that need to be reverted to the state
    // auto = std::map<std::pair<std::string, std::vector<std::string> >, std::vector<rosplan_knowledge_msgs::KnowledgeItem> >::iterator

    // NOTE: this method only allows to revert actions that you have applied in the previously, if you wish
    // to revert actions based on effects the use the revertActionBlind() method

    // select correct map
    if(action_start) {
        // revert action start

        auto it = start_sim_actions_map_.find(std::pair<std::string, std::vector<std::string> >(action_name, params));
        // check if key was found in map
        if(it == start_sim_actions_map_.end()) {
            ROS_ERROR("failed to find action start in map, have you applied the action before?");
            return false;
        } else {
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> effects_to_revert = it->second;
            return reverseEffectsFromKIA(effects_to_revert);
        }
    }
    else {
        // revert action end

        auto it = end_sim_actions_map_.find(std::pair<std::string, std::vector<std::string> >(action_name, params));
        // check if key was found in map
        if(it == end_sim_actions_map_.end()) {
            ROS_ERROR("failed to find action end in map, have you applied the action before?");
            return false;
        } else {
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> effects_to_revert = it->second;
            return reverseEffectsFromKIA(effects_to_revert);
        }
    }

    // should never reach here actually
    return false;
}

bool ActionSimulator::revertActionBlind(std::string &action_name, std::vector<std::string> &params, bool action_start)
{
    // inverse of apply action, used for backtracking purposes
    // revert means: check action effects and revert them (delete positive effects, add negative effects)

    rosplan_knowledge_msgs::DomainOperator op;
    std::map<std::string, std::string> ground_dictionary;

    // get ground dictionary, a map between keys and grounded action parameters
    if(!computeGroundDictionary(action_name, params, ground_dictionary, op)) {
        ROS_WARN("failed to compute ground dictionary");
        return false;
    }

    // query action effects from domain operator
    if(action_start)
    {
        // revert action start effects

        // iterate over positive start action effects and apply to KB
        for(auto it=op.at_start_add_effects.begin(); it!=op.at_start_add_effects.end(); it++)
            removeFactInternal(it->name, groundParams(*it, ground_dictionary));

        // iterate over negative start action effects and apply to KB
        for(auto it=op.at_start_del_effects.begin(); it!=op.at_start_del_effects.end(); it++) {
            addFactInternal(it->name, groundParams(*it, ground_dictionary));
        }
    }
    else
    {
        // revert action end effects

        // iterate over positive start action effects and apply to KB
        for(auto it=op.at_end_add_effects.begin(); it!=op.at_end_add_effects.end(); it++)
            removeFactInternal(it->name, groundParams(*it, ground_dictionary));

        // iterate over negative start action effects and apply to KB
        for(auto it=op.at_end_del_effects.begin(); it!=op.at_end_del_effects.end(); it++) {
            addFactInternal(it->name, groundParams(*it, ground_dictionary));
        }
    }

    return true;
}

bool ActionSimulator::revertActionStart(std::string &action_name, std::vector<std::string> &params)
{
    // overloaded function to revert action start effects
    return revertAction(action_name, params, true);
}

bool ActionSimulator::revertActionEnd(std::string &action_name, std::vector<std::string> &params)
{
    // overloaded function to revert action end effects
    return revertAction(action_name, params, false);
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
            ROS_DEBUG("missing goal : (%s)", convertPredToString(*it).c_str());
            goal_state_reached =  false;
        }
    }

    return goal_state_reached;
}

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "action_simulator_tester");
    ROS_DEBUG("Node is going to initialize...");
    // create object of the node class (ActionSimulator)
    ActionSimulator action_simulator_tester(true, true);
    ROS_DEBUG("Node initialized.");

    // SNIPPETS: Example of how to use this library

    // snippet: get operator names
    std::vector<std::string> operator_names;
    action_simulator_tester.getOperatorNames(operator_names);
    ROS_DEBUG("get operator names:");
    ROS_DEBUG("===============");
    for(auto it=operator_names.begin(); it != operator_names.end(); it++)
    {
        ROS_DEBUG("%s", it->c_str());
    }

    // snippet: get all domain predicate names
    std::vector<std::string> domain_predicates;
    action_simulator_tester.getAllPredicateNames(domain_predicates);
    ROS_DEBUG("get all predicate names:");
    ROS_DEBUG("================");
    for(auto it=domain_predicates.begin(); it != domain_predicates.end(); it++)
    {
        ROS_DEBUG("%s", it->c_str());
    }

    // save all predicates to internal KB
    action_simulator_tester.saveKBSnapshot();

    // snippet: print internal KB facts
    ROS_DEBUG("print internal KB facts:");
    ROS_DEBUG("================");
    action_simulator_tester.printInternalKBFacts();

    // snippet: print internal KB goals
    ROS_DEBUG("print internal KB goals:");
    ROS_DEBUG("================");
    action_simulator_tester.printInternalKBGoals();

    // snippet: find fact in KB (with no args)
    ROS_DEBUG("find fact in internal KB no args:");
    ROS_DEBUG("================");
    std::string predicate_name2 = "person_descending";
    if(action_simulator_tester.findFactInternal(predicate_name2))
        ROS_DEBUG("predicate : (person_descending) found in internal KB");
    else
        ROS_DEBUG("predicate : (person_descending) not found in internal KB");
    // alternative option
    ROS_DEBUG("find fact in internal KB no args (alternative option):");
    ROS_DEBUG("================");
    std::string predicate_name3 = "person_descending";
    std::vector<std::string> args1 = {""};
    if(action_simulator_tester.findFactInternal(predicate_name3, args1))
        ROS_DEBUG("predicate : (person_descending) found in internal KB");
    else
        ROS_DEBUG("predicate : (person_descending) not found in internal KB");

    // snippet: find fact in KB (with args)
    ROS_DEBUG("find fact in internal KB with args:");
    ROS_DEBUG("================");
    std::string predicate_name4 = "has_driver_license";
    std::vector<std::string> args2 = {"batdad"};
    if(action_simulator_tester.findFactInternal(predicate_name4, args2))
        ROS_DEBUG("predicate : (has_driver_license batdad) found in internal KB");
    else
        ROS_DEBUG("predicate : (has_driver_license batdad) not found in internal KB");

    // snippet: remove predicate from KB (with args)
    std::string predicate_name = "has_driver_license";
    std::vector<std::string> args3 = {"batdad"};
    action_simulator_tester.removeFactInternal(predicate_name, args3);
    ROS_DEBUG("test: remove predicate with args (has_driver_license batdad):");
    ROS_DEBUG("================");
    action_simulator_tester.printInternalKBFacts();

    // snippet: remove predicate from KB (no args)
    std::string predicate_name5 = "person_descending";
    std::vector<std::string> args4 = {""};
    ROS_DEBUG("test: remove predicate with no args (person_descending):");
    ROS_DEBUG("================");
    action_simulator_tester.removeFactInternal(predicate_name5, args4);
    action_simulator_tester.printInternalKBFacts();

    // snippet: see if action is applicable
    std::string action_name = "get_down_from_car";
    std::vector<std::string> params = {"batdad","car","ben_school"}; // person, car, location
    ROS_DEBUG("check if action is applicable: (get_down_from_car batdad car ben_school), expected outcome is true");
    ROS_DEBUG("================");
    if(action_simulator_tester.isActionApplicable(action_name, params))
        ROS_DEBUG("action is applicable!");
    else
        ROS_DEBUG("action is not applicable");

    // snippet: simulate an action start
    ROS_DEBUG("simulate action start: (get_down_from_car batdad car ben_school)");
    ROS_DEBUG("================");
    ROS_DEBUG("KB before simulation");
    action_simulator_tester.printInternalKBFacts();
    std::string action_name2 = "get_down_from_car";
    std::vector<std::string> params2 = {"batdad","car","ben_school"};
    action_simulator_tester.simulateActionStart(action_name2, params2);
    ROS_DEBUG("KB after simulation");
    action_simulator_tester.printInternalKBFacts();

    // snippet: test areGoalsAchieved()
    ROS_DEBUG("Check if goals are achieved:");
    ROS_DEBUG("================");
    ROS_DEBUG("KB goals:");
    action_simulator_tester.printInternalKBGoals();
    ROS_DEBUG("KB facts:");
    action_simulator_tester.printInternalKBFacts();
    if(action_simulator_tester.areGoalsAchieved())
        ROS_DEBUG("goals achieved");
    else
        ROS_DEBUG("goals not achieved");

    // snippet: test revert actions
    ROS_DEBUG("Check reverting action: (get_down_from_car batdad car ben_school)");
    ROS_DEBUG("================");
    ROS_DEBUG("KB facts:");
    action_simulator_tester.printInternalKBFacts();
    std::string action_name3 = "get_down_from_car";
    std::vector<std::string> params3 = {"batdad","car","ben_school"}; // person, car, location
    if(action_simulator_tester.revertActionStart(action_name3, params3))
        ROS_DEBUG("action reverted succesfully");
    else
        ROS_ERROR("failed to revert action");
    ROS_DEBUG("KB facts after reverting action:");
    action_simulator_tester.printInternalKBFacts();

    return 0;
}
