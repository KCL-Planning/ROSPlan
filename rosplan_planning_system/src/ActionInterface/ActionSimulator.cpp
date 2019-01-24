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

#include <rosplan_action_interface/ActionSimulator.h>

ActionSimulator::ActionSimulator() : nh_("~")
{
    // for now we call from constructor, in future we can remove
    prepareServices();
}

ActionSimulator::~ActionSimulator() {}

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

bool ActionSimulator::saveOperatorNames()
{
    // get domain operator details, save them in member variable
    return getOperatorNames(operator_names_);
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
        op = srv.response.op;
    }
    else {
        ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for operator details, %s", od_srv_client_.getService().c_str());
        return false;
    }

    return true;
}

bool ActionSimulator::getSomeOperatorDetails(std::vector<std::string> &operator_names,
        std::vector<rosplan_knowledge_msgs::DomainOperator> &domain_operator_details)
{
    // fetch some domain operator details (depending on operator_names) and store them locally for future use

    // make sure input is not empty
    if(!operator_names.size() > 0) {
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

bool ActionSimulator::getAllOperatorDetails(std::vector<rosplan_knowledge_msgs::DomainOperator> &domain_operator_details)
{
    // fetch all domain operatos names, call getSomeOperatorDetails()

    // check if we have stored operator names before
    if(operator_names_.size() > 0) {
        return getSomeOperatorDetails(operator_names_, domain_operator_details);
    }
    else{
        // get all operator names, store them in operator_names
        std::vector<std::string> operator_names;
        getOperatorNames(operator_names);
        return getSomeOperatorDetails(operator_names, domain_operator_details);
    }
}

bool ActionSimulator::saveAllOperatorDetails()
{
    // get all domain operator details and store them in member variable for future use
    return getAllOperatorDetails(domain_operator_details_);
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

bool ActionSimulator::saveAllPredicatesDetails()
{
    // get all domain predicate details and store them in member variable for future use
    return getPredicatesDetails(domain_predicate_details_);
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

bool ActionSimulator::saveAllPredicateNames()
{
    // get all domain predicates details, extract their names and store them in member variable list
    return getAllPredicateNames(domain_predicates_);
}

bool ActionSimulator::makeOperatorDetailsMap()
{
    // make a map between operator_names_ and domain_operator_details_

    // make sure inputs are not empty
    if(!operator_names_.size() > 0 && !domain_operator_details_.size() > 0) {
        ROS_ERROR("operator_names_ or domain_operator_details_ is empty, try calling initInternalKB() first");
        return false;
    }

    // iterator over operator_names_
    // delete previous data if any
    domain_operator_map_.clear();
    auto doit = domain_operator_details_.begin();
    for(auto it=operator_names_.begin(); it!=operator_names_.end(); it++) {
        domain_operator_map_.insert(std::pair<std::string, rosplan_knowledge_msgs::DomainOperator>(*it, *doit++));
    }

    return true;
}

bool ActionSimulator::getGroundedPredicates(std::string &predicate_name, std::vector<rosplan_knowledge_msgs::KnowledgeItem> &facts)
{
    // get all grounded facts related with a specific predicate_name

    // wait for service
    if(!checkServiceExistance(sp_srv_client_))
        return false;

    // call srv
    rosplan_knowledge_msgs::GetAttributeService srv;

    srv.request.predicate_name = predicate_name;

    if(sp_srv_client_.call(srv))
    {
        // clear previous data, if any
        facts.clear();

        for(auto it=srv.response.attributes.begin(); it!= srv.response.attributes.end(); it++) {
            // return by reference a list of domain operator names
            facts.push_back(*it);
        }
    }
    else {
        ROS_ERROR("KCL: (ActionSimulator) Could not call Knowledge Base service: %s", sp_srv_client_.getService().c_str());
        return false;
    }

    return true;
}

bool ActionSimulator::getAllGroundedPredicates(std::vector<rosplan_knowledge_msgs::KnowledgeItem> &all_facts)
{
    // get all grounded facts in the KB

    // get all predicate names
    std::vector<std::string> domain_predicates;
    if(!getAllPredicateNames(domain_predicates))
        return false;

    // delete previous data if any
    all_facts.clear();

    // iterate over the domain predicates names
    for(auto it = domain_predicates.begin(); it != domain_predicates.end();it++) {
        // call service to get all facts related with this name
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> facts;
        getGroundedPredicates(*it, facts);

        // iterate over the facts and save them all in one big combined KB
        for(auto fit = facts.begin(); fit != facts.end(); fit++) {
            all_facts.push_back(*fit);
        }
    }

    return true;
}

bool ActionSimulator::saveAllGroundedPredicates()
{
    // get all grounded predicates in KB, save in member variable
    return getAllGroundedPredicates(internal_kb_);
}

bool ActionSimulator::initInternalKB()
{
    // call all related functions that populate required member variables to simulate actions and more

    // get all predicate details, save them in domain_predicate_details_
    if(!saveAllPredicatesDetails())
        return false;

    // get a list of all operator names, save in operator_names_
    if(!saveOperatorNames())
        return false;

    // get a list of all domain predicates, save in domain_predicates_
    if(!saveAllPredicateNames())
        return false;

    // get a list of all domain operator details, save in domain_operator_details_
    if(!saveAllOperatorDetails())
        return false;

    // make map between operator_names_ and domain_operator_details_, save in domain_operator_map_
    if(!makeOperatorDetailsMap())
        return false;

    // get all state grounded predicates, save them in internal_kb_
    if(!saveAllGroundedPredicates())
        return false;

    // make internal_kb_ bkp
    backupInternalKB();

    return true;
}

void ActionSimulator::deleteAllInternalKB()
{
    domain_predicate_details_.clear();
    operator_names_.clear();
    domain_predicates_.clear();
    domain_operator_details_.clear();
    domain_operator_map_.clear();
    internal_kb_.clear();
    internal_kb_.clear();
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

bool ActionSimulator::printInternalKB()
{
    // print all predicates in internal KB

    // saveAllGroundedPredicates() needs to be called first

    if(!internal_kb_.size() > 0) {
        ROS_ERROR("internal KB is empty, failed to print (try calling initInternalKB() first?)");
        return false;
    }

    ROS_INFO("....internal KB.....");

    for(auto it=internal_kb_.begin(); it!=internal_kb_.end(); it++)
    {
        std::stringstream predicate_ss;

        predicate_ss << it->attribute_name;

        for(auto vit=it->values.begin(); vit!=it->values.end(); vit++) {
            predicate_ss << " ";
            predicate_ss << vit->value;
        }

        if(it->is_negative) {
            ROS_INFO("(not(%s))", predicate_ss.str().c_str());
        }
        else {
            ROS_INFO("(%s)", predicate_ss.str().c_str());
        }
    }

    ROS_INFO("...................");

    return true;
}

void ActionSimulator::backupInternalKB()
{
    // save internal KB in second member variable for reset purposes: in case user wants to go
    // back to previoud KB version, without having to perform another service call
    internal_kb_bkp_ = internal_kb_;
}

bool ActionSimulator::findFactInternal(std::string &predicate_name, std::vector<std::string> &args,
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator &kiit)
{
    // find predicate inside KB, return by reference an iterator to the element

    // check that internal KB has at least 1 element
    if(!internal_kb_.size() > 0) {
        ROS_ERROR("internal_kb_ is empty");
        return false;
    }

    // flag to indicate that the predicate was found
    bool found = false;

    // iterate over internal KB
    for(auto it=internal_kb_.begin(); it!=internal_kb_.end(); it++)
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
                found = true;
                // in theory there should'nt be duplicate predicates in KB, so no need to look for more
                break;
            }
            else
                continue;
        }

        // compare arguments one by one
        for(auto ait=args.begin(); ait!=args.end(); ait++)
            if(!(*ait == it->values.at(ait - args.begin()).value))
                continue;

        // found element (has args), return iterator "it"
        kiit = it;
        found = true;
        // in theory there should'nt be duplicate predicates in KB, so no need to look for more
        break;
    }

    return found;
}

bool ActionSimulator::findFactInternal(std::string &predicate_name, std::vector<std::string> &args)
{
    // overloaded function offered to call findFactInternal() when we don't care about the returned iterator
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kiit;
    return findFactInternal(predicate_name, args, kiit);
}

bool ActionSimulator::findFactInternal(std::string &predicate_name)
{
    // overloaded function offered to call findFactInternal() when we don't care about the returned iterator
    // and args are empty
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kiit;
    std::vector<std::string> args = {""};
    return findFactInternal(predicate_name, args, kiit);
}

bool ActionSimulator::findFactInternal(std::vector<std::string> &predicate_name_and_params)
{
    // overloaded function offered to call findFactInternal() when we don't care about the returned iterator
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

bool ActionSimulator::findFactInternal(rosplan_knowledge_msgs::DomainFormula &predicate)
{
    // overloaded function that alows to find predicate in KB with an input DomainFormula (which can store a predicate)

    // extract params from DomainFormula
    std::vector<std::string> params;
    for(auto it=predicate.typed_parameters.begin(); it!=predicate.typed_parameters.end(); it++)
        params.push_back(it->value);

    return findFactInternal(predicate.name, params);
}

bool ActionSimulator::removeFactInternal(std::string &predicate_name, std::vector<std::string> &args)
{
    // remove fact from internal KB

    // find element in KB and get a pointer to it
    std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator it;

    if(findFactInternal(predicate_name, args, it)) {
        // element exists, delete knowledge item from internal KB
        internal_kb_.erase(it);
    }
    else {
        // element does not exist
        ROS_ERROR("Could not find predicate in internal KB (%s), while trying to delete",
                  convertPredToString(predicate_name, args).c_str());
        return false;
    }

    return true;
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

void ActionSimulator::addFactInternal(std::string &predicate_name, std::vector<std::string> &params)
{
    // add fact to internal KB

    // make sure fact is not already there
    if(findFactInternal(predicate_name, params)) {
        // element exists
        ROS_WARN("Tried to add fact twice, doing nothing... (%s)", convertPredToString(predicate_name, params).c_str());
        return;
    }

    // internal_kb_ is of type -> std::vector<rosplan_knowledge_msgs::KnowledgeItem>

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
    internal_kb_.push_back(knowledge_item);
}

void ActionSimulator::addFactInternal(rosplan_knowledge_msgs::DomainFormula &predicate)
{
    // overloaded method that allows to add facts to internal KB with DomainFormula input

    // extract params from predicate input
    std::vector<std::string> params;
    for(auto it=predicate.typed_parameters.begin(); it!=predicate.typed_parameters.end(); it++) {
        params.push_back(it->value);
    }

    // call original method with adjusted arguments
    return addFactInternal(predicate.name, params);
}

std::vector<std::string> ActionSimulator::GroundParams(rosplan_knowledge_msgs::DomainFormula &ungrounded_precondition,
    std::map<std::string, std::string> &ground_dictionary)
{
    // receive an operator and a dictionary of key values, return grounded predicate

    std::vector<std::string> grounded_fact;

    // insert as first element the predicate name
    grounded_fact.push_back(ungrounded_precondition.name);

    // iterate over the ungrounded params
    for(auto it=ungrounded_precondition.typed_parameters.begin(); it!=ungrounded_precondition.typed_parameters.end(); it++)
        grounded_fact.push_back(ground_dictionary.find(it->key)->second);

    return grounded_fact;
}

bool ActionSimulator::isActionAplicable(std::string &action_name, std::vector<std::string> &params)
{
    // check if action preconditions are consistent with internal KB information

    // ensure domain_operator_map_ has data
    if(!domain_operator_map_.size() > 0) {
        ROS_ERROR("domain_operator_map_ is empty, try calling initInternalKB() first");
        return false;
    }

    // get operator from action name (std::map of names, operators)
    auto oit = domain_operator_map_.find(action_name)->second;

    // construct ground dictionary from params
    std::map<std::string, std::string> ground_dictionary; // key value
    // ensure they have the same size
    if(!(oit.formula.typed_parameters.size() == params.size())) {
        ROS_ERROR("wrong parameters, size of operator parameters and params size do not match");
        return false;
    }
    for(auto it=oit.formula.typed_parameters.begin(); it!=oit.formula.typed_parameters.end(); it++)
        ground_dictionary.insert(std::pair<std::string, std::string>(it->key, params.at(std::distance(oit.formula.typed_parameters.begin(), it))));

    // check action preconditions

    // iterate over ungrounded positive preconditions, find in KB
    for(auto it=oit.at_start_simple_condition.begin(); it!=oit.at_start_simple_condition.end(); it++) {
        // if not found, action is not aplicable
        std::vector<std::string> gp = GroundParams(*it, ground_dictionary);
        if(!findFactInternal(gp)) // "it" is in DomainFormula format
            return false;
    }

    // iterate over ungrounded overall conditions, find in KB
    for(auto it=oit.over_all_simple_condition.begin(); it!=oit.over_all_simple_condition.end(); it++) {
        // if not found, action is not aplicable
        std::vector<std::string> gp = GroundParams(*it, ground_dictionary);
        if(!findFactInternal(gp))
            return false;
    }

    // iterate over ungrounded at_start_neg_condition, make sure they are not in KB
    for(auto it=oit.at_start_neg_condition.begin(); it!=oit.at_start_neg_condition.end(); it++) {
        // if found, action is not aplicable
        std::vector<std::string> gp = GroundParams(*it, ground_dictionary);
        if(findFactInternal(gp))
            return false;
    }

    return true;
}

bool ActionSimulator::simulateAction(std::string &action_name, std::vector<std::string> &params, bool action_start)
{
    // apply delete and add list to KB current state

    // ensure domain_operator_details_ has data
    if(!domain_operator_details_.size() > 0) {
        ROS_ERROR("domain_operator_details_ is empty, try calling initInternalKB() first");
        return false;
    }

    // check if action is applicable
    if(!isActionAplicable(action_name, params)) {
        ROS_ERROR("action is not applicable, will not simulate");
        return false;
    }

    // query action effects

    // get operator from action name (std::map of names, operators)
    auto oit = domain_operator_map_.find(action_name)->second;

    if(action_start)
    {
        // action start effects

        // iterate over positive start action effects and apply to KB
        for(auto it=oit.at_start_add_effects.begin(); it!=oit.at_start_add_effects.end(); it++)
            addFactInternal(*it);

        // iterate over negative start action effects and apply to KB
        for(auto it=oit.at_start_del_effects.begin(); it!=oit.at_start_del_effects.end(); it++) {
            if(!removeFactInternal(*it))
                return false;
        }
    }
    else
    {
        // action end effects

        // iterate over positive start action effects and apply to KB
        for(auto it=oit.at_end_add_effects.begin(); it!=oit.at_end_add_effects.end(); it++)
            addFactInternal(*it);

        // iterate over negative start action effects and apply to KB
        for(auto it=oit.at_end_del_effects.begin(); it!=oit.at_end_del_effects.end(); it++) {
            if(!removeFactInternal(*it))
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

int main(int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "action_simulator_tester");
    ROS_INFO("Node is going to initialize...");
    // create object of the node class (ActionSimulator)
    ActionSimulator action_simulator_tester;
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
    action_simulator_tester.saveAllGroundedPredicates();

    // snippet: print internal KB
    ROS_INFO("print internal KB:");
    ROS_INFO("================");
    action_simulator_tester.printInternalKB();

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
    ROS_INFO("internal KB after having removed predicate (has_driver_license batdad):");
    ROS_INFO("================");
    action_simulator_tester.printInternalKB();

    // snippet: remove predicate from KB (no args)
    std::string predicate_name5 = "person_descending";
    std::vector<std::string> args4 = {""};
    ROS_INFO("internal KB after having removed predicate (person_descending):");
    ROS_INFO("================");
    action_simulator_tester.removeFactInternal(predicate_name5, args4);
    action_simulator_tester.printInternalKB();

    // snippet: see if action is aplicable
    std::string action_name = "get_down_from_car";
    std::vector<std::string> params = {"batdad","car","ben_school"}; // person, car, location
    ROS_INFO("check if action is applicable: (get_down_from_car batdad car ben_school), expected outcome is true");
    ROS_INFO("================");
    action_simulator_tester.initInternalKB();
    if(action_simulator_tester.isActionAplicable(action_name, params))
        ROS_INFO("action is applicable!");
    else
        ROS_INFO("action is not applicable");

    // snippet: simulate an action
    ROS_INFO("KB before simulating action:");
    ROS_INFO("================");
    action_simulator_tester.printInternalKB();
    ROS_INFO("action simulation: (get_down_from_car batdad car ben_school)");
    ROS_INFO("================");
    std::string action_name2 = "get_down_from_car";
    std::vector<std::string> params2 = {"batdad","car","ben_school"};
    // action_simulator_tester.initInternalKB(); // call only once, here it has been called before, therefore not calling
    action_simulator_tester.simulateActionStart(action_name2, params2);
    // print to see the difference in KB
    ROS_INFO("KB after simulating action:");
    ROS_INFO("================");
    action_simulator_tester.printInternalKB();

    return 0;
}
