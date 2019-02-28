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

#include <ros/ros.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorService.h>
#include <rosplan_knowledge_msgs/DomainOperator.h>
#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <diagnostic_msgs/KeyValue.h>

class ActionSimulator
{
    public:

        /**
         * @brief empty constructor
         */
        ActionSimulator();

        /**
         * @brief constructor, provide the option not to mirror any KB at startup
         * @param mirror_KB_at_startup if true, a query is performed to real KB domain details and values stored internally
         * @param mirror_facts_and_goals if true, facts and goals are fetched from real KB and stored internally
         */
        ActionSimulator(bool mirror_KB_at_startup, bool mirror_facts_and_goals);

        /**
         * @brief destructor
         */
        ~ActionSimulator();

        /**
         * @brief prepare services to connect to real KB and get domain data
         */
        void init();

        /**
         * @brief returns by reference a list of domain operator names
         * @param operator_names return value will be written here, pass an empty list of strings
         * @return true if KB domain service was available, false otherwise
         */
        bool getOperatorNames(std::vector<std::string> &operator_names);

        /**
         * @brief get from KB a single domain operator detail, based on input name (operator_name)
         * @param operator_name the name of the operator (action) from which we want to query it's details
         * @param op return value gets written here by reference
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool getOperatorDetails(std::string &operator_name, rosplan_knowledge_msgs::DomainOperator &op);

        /**
         * @brief fetch some domain operator details (depending on operator_names) and store them locally for future use
         * @param operator_names a list of operators from which we want to query their details
         * @param domain_operator_details return value gets written here by reference
         * @return true if input operator_names size is greater than 0, false otherwise
         */
        bool getSomeOperatorDetails(std::vector<std::string> &operator_names,
            std::vector<rosplan_knowledge_msgs::DomainOperator> &domain_operator_details);

        /**
         * @brief fetch all domain operator details and store them locally for future use
         * @param domain_operator_details return value gets written here by reference
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool getAllOperatorDetails(std::vector<rosplan_knowledge_msgs::DomainOperator> &domain_operator_details);

        /**
         * @brief get all domain predicate details
         * @param domain_predicate_details return value gets written here by reference
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool getPredicatesDetails(std::vector<rosplan_knowledge_msgs::DomainFormula> &domain_predicate_details);

        /**
         * @brief get all domain predicates details, extract their names, return them by reference
         * @param domain_predicates return value gets written here by reference
         * @return true if service was found and it's call was successful, false otherwise
         */
        bool getAllPredicateNames(std::vector<std::string> &domain_predicates);

        /**
         * @brief get all knowledge items in KB, can be facts or goals
         * @param srv_client a GetAttributeService KB client, can be either kb_name/state/propositions or kb_name/state/goals
         * @param knowledge return type gets written in this variable by reference
         * @return true if able to fetch knowledge from KB, false otherwise
         */
        bool getAllKnowledgeItems(ros::ServiceClient &srv_client,
            std::vector<rosplan_knowledge_msgs::KnowledgeItem> &knowledge);

        /**
         * @brief get all grounded facts in the KB, performs multiple calls to getGroundedPredicates()
         * @param all_facts return value gets written here by reference
         * @return true if able to fetch grounded facts from KB, false otherwise
         */
        bool getAllGroundedFacts(std::vector<rosplan_knowledge_msgs::KnowledgeItem> &all_facts);

        /**
         * @brief fetch goals and facts from the real KB, save them in internal KB
         * @return true if call to real KB is successful, false otherwise
         */
        bool saveKBSnapshot();

        /**
         * @brief handy function to help facilitate the printing of predicates
         * @param predicate_name string with the name of the predicate
         * @param params list of predicate arguments
         * @return string with the predicate ready to be printed
         */
        std::string convertPredToString(std::string &predicate_name, std::vector<std::string> &params);

        /**
         * @brief handy function to help facilitate the printing of predicates
         * @param knowledge_item predicate to be converted to string given in KnowledgeItem format
         * @return string with the predicate ready to be printed
         */
        std::string convertPredToString(rosplan_knowledge_msgs::KnowledgeItem &knowledge_item);

        /**
         * @brief print all elements in a KnowledgeItem array (used for printing all facts or goals)
         * @param ki_array either kb_goals_ or kb_facts_, in general the array to be printed
         */
        void printArrayKI(std::vector<rosplan_knowledge_msgs::KnowledgeItem> ki_array, std::string header_msg);

        /**
         * @brief print all facts in internal KB
         * @return false if kb_facts_ is empty, true otherwise
         */
        void printInternalKBFacts();

        /**
         * @brief print all goals in internal KB
         * @return false if kb_goals_ is empty, true otherwise
         */
        void printInternalKBGoals();

        /**
         * @brief save internal KB in second member variable for reset purposes: in case user wants to go
         * back to previoud KB version, without having to perform another service call
         */
        void backupInternalKB();

        /**
         * @brief find predicate (fact or goal) inside knowledge item array, return by reference an iterator to the element
         * @param predicate_name
         * @param args list of predicate parameters
         * @param kiit return value gets written here by reference, an iterator to the element (if it was found)
         * @param item_array the list in which the predicate will be searched
         */
        bool findKnowledgeItem(std::string &predicate_name, std::vector<std::string> &args,
                        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator &kiit,
                        std::vector<rosplan_knowledge_msgs::KnowledgeItem> &item_array);

        /**
         * @brief find either a predicate (goal or fact) with KnowledgeItem input
         * @param predicate fact or goal to be searched in KnowledgeItem format
         * @param is_fact if true then it will search in fact KB (kb_facts_), otherwise it will search in goals KB (kb_goals_)
         * @return true if found, false otherwise
         */
        bool findPredicateInternal(rosplan_knowledge_msgs::KnowledgeItem &predicate, bool is_fact);

        /**
         * @brief find (grounded) predicate inside KB, return by reference an iterator to the element
         * @param predicate_name the name of the predicate to search in the KB
         * @param args the predicate grounded parameters
         * @param kiit return value gets written here by reference
         * @return true if predicate was found, false otherwise
         */
        bool findFactInternal(std::string &predicate_name, std::vector<std::string> &args,
                std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator &kiit);

        /**
         * @brief overloaded function offered to call FindFactInternal() when we don't care about the returned iterator
         * @param predicate_name the name of the predicate to search in the KB
         * @param args the predicate grounded parameters
         * @return true if predicate was found, false otherwise
         */
        bool findFactInternal(std::string &predicate_name, std::vector<std::string> args);

        /**
         * @brief overloaded function offered to call FindFactInternal() when we don't care about the returned iterator
         * and args are empty
         * @param predicate_name the name of the predicate to search in the KB
         * @return true if predicate was found, false otherwise
         */
        bool findFactInternal(std::string &predicate_name);

        /**
         * @brief overloaded function offered to call FindFactInternal() when we don't care about the returned iterator
         * and args are empty and we have a single vector representing the predicate where the first element is the predicate name
         * @param predicate_name_and_params first element is the predicate name, succesive elements are predicate arguments
         * @return true if predicate was found, false otherwise
         */
        bool findFactInternal(std::vector<std::string> predicate_name_and_params);

        /**
         * @brief overloaded function that allows to find predicate in KB with an input KnowledgeItem
         * @param fact proposition in form of KnowledgeItem
         * @return true if predicate was found, false otherwise
         */
        bool findFactInternal(rosplan_knowledge_msgs::KnowledgeItem &fact);

        /**
         * @brief allows to find a single goal in internal KB
         * @param predicate_name goal predicate name
         * @param args goal predicate parameters if any
         * @param kiit return value gets written here, an iterator to the goal, if found
         * @return true if goal was found, false otherwise
         */
        bool findGoalInternal(std::string &predicate_name, std::vector<std::string> &args,
            std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator &kiit);

        /**
         * @brief allows to find a single goal in internal KB with KnowledgeItem as input
         * @param predicate_name goal predicate name
         * @param args goal predicate parameters if any
         * @return true if goal was found, false otherwise
         */
        bool findGoalInternal(std::string &predicate_name, std::vector<std::string> &args);

        /**
         * @brief allows to find a single goal in internal KB with KnowledgeItem as input
         * @param goal proposition in form of KnowledgeItem
         * @return true if goal was found, false otherwise
         */
        bool findGoalInternal(rosplan_knowledge_msgs::KnowledgeItem &goal);

        /**
         * @brief remove predicate from KnowledgeItem item_array
         * @param predicate_name the name of the predicate to be removed from KB
         * @param args the grounded predicate parameters
         * @param item_array KnowledgeItem array from which to remove the predicate (goal or fact)
         * @return true if predicate was found, false otherwise
         */
        bool removePredicateInternal(std::string &predicate_name, std::vector<std::string> &args,
                std::vector<rosplan_knowledge_msgs::KnowledgeItem> &item_array);

        /**
         * @brief remove fact from internal KB
         * @param predicate_name the name of the predicate to be removed from KB
         * @param args the grounded predicate parameters
         * @return true if predicate was found, false otherwise
         */
        bool removeFactInternal(std::string &predicate_name, std::vector<std::string> args);

        /**
         * @brief remove fact from internal KB, this in an overloaded method that
         * accepts the predicate as a single list of strings
         * @param predicate_and_args single list of strings where the predicate can be expressed
         * @return true if predicate was found, false otherwise
         */
        bool removeFactInternal(std::vector<std::string> &predicate_and_args);

        /**
         * @brief remove fact from internal KB, this in an overloaded method that
         * accepts the predicate in DomainFormula format
         * @param predicate the predicate that needs to be removed from KB in DomainFormula format
         * @return true if predicate was found, false otherwise
         */
        bool removeFactInternal(rosplan_knowledge_msgs::DomainFormula &predicate);

        /**
         * @brief remove a goal from internal goal KB (kb_goals_)
         * @param predicate_name the name of the predicate that needs to be removed
         * @param args the grounded predicate parameters
         */
        bool removeGoalInternal(std::string &predicate_name, std::vector<std::string> &args);

        /**
         * @brief add fact to internal KB, warning: does not fill keys on knowledge items, it leaves the keys empty
         * @param predicate_name the name of the predicate to be added to internal KB
         * @param params the parameters of the predicate to be added to internal KB
         * @return true if fact was added and did not belonged already to KB, false if fact was already
         * in KB and therefore it did not added
         */
        bool addFactInternal(std::string &predicate_name, std::vector<std::string> params);

        /**
         * @brief receive an operator and a dictionary of key values, return grounded predicate parameters
         * @param ungrounded_precondition taken from operator details, this is a DomainFormula precondition
         * @param ground_dictionary std map of string to string with key value information: key, grounded parameter
         * @return a grounded predicate, in form of list of string
         */
        std::vector<std::string> groundParams(rosplan_knowledge_msgs::DomainFormula ungrounded_precondition,
            std::map<std::string, std::string> &ground_dictionary);

        /**
         * @brief get domain operator corresponding to the received action, make a map between keys and grounded action params
         * @param action_name name of the action from which to construct the ground dictionary
         * @param params grounded action parameters
         * @param ground_dictionary return value gets written here by reference
         * @param op return value gets written here by reference, pass empty value
         * @return true if success, false if failure
         */
        bool computeGroundDictionary(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary, rosplan_knowledge_msgs::DomainOperator &op);

        /**
         * @brief query internal KB to fetch the probability of that fact being true
         * @param action_name the name of the action you want its probability
         * @param params the grounded parameters of the above action_name
         * @return a double: the probability of that fact being true
         */
        double getFactProbability(std::string &action_name, std::vector<std::string> &params);

        /**
         * @brief generic helper function to avoid code repetition, it receives a domain formula.
         * e.g. at_start_simple_condition or at_end_neg_condition, iterates over the conditions and
         * returns if they are satisfied or not
         * @param positive_conditions e.g. at_end_neg_condition -> pass a false, at_start_simple_condition -> pass true
         * @param df typically you do e.g. op.at_end_neg_condition, from a domain operator
         * @param ground_dictionary a map between ungrounded keys with its correspondent grounded values
         * @param combined_probability return value gets written here by reference, if needed (optional) this
         * function can calculate the probability of the action being applicable (by multiplying the probability of relevant facts being true)
         * @return true if all conditions are met, false otherwise
         */
        bool checkConditions(bool positive_conditions, std::vector<rosplan_knowledge_msgs::DomainFormula> &df,
        std::map<std::string, std::string> &ground_dictionary, double &combined_probability);

        /**
         * @brief check if action start/end/overall preconditions are consistent with internal KB information
         * and return by reference the ground dictionary for simulation action purposes
         * and return the probability of this action to succeed, based on the combined probability of all facts being true
         * @param action_name the name of the action to check if preconditions are met in KB
         * @param params action grounded parameters
         * @param action_start if true, at start preconditions are checked
         * @param action_overall if true, overall preconditions are checked
         * @param action_end if true, at end preconditions are checked
         * @param ground_dictionary return value gets written here by reference
         * @param combined_probability return value gets written here by reference, a multiplication of all involved facts
         * probabilities of those facts being true
         * @return true if action start/end/overall is applicable, false otherwise
         */
        bool isActionApplicable(std::string &action_name, std::vector<std::string> &params, bool action_start, bool action_overall,
                bool action_end, std::map<std::string, std::string> &ground_dictionary, double &combined_probability);

        /**
         * @brief overloaded function that checks if all action preconditions (start, end, overall) are consistent with
         * internal KB information and return by reference the ground dictionary for simulation action purposes
         * @param action_name the name of the action to check if preconditions are met in KB
         * @param params action grounded parameters
         * @param ground_dictionary return value gets written here by reference
         * @return true if action is applicable, false otherwise
         */
        bool isActionApplicable(std::string &action_name, std::vector<std::string> &params,
                std::map<std::string, std::string> &ground_dictionary);

        /**
         * @brief overloaded function that checks if all action preconditions (start, end, overall) are consistent with
         * internal KB information
         * @param action_name the name of the action to check if preconditions are met in KB
         * @param params action grounded parameters
         * @return true if action is applicable, false otherwise
         */
        bool isActionApplicable(std::string &action_name, std::vector<std::string> &params);

        /**
         * @brief overloaded function that checks if action start preconditions are consistent with internal KB information
         * and return by reference the ground dictionary for simulation action purposes
         * @param action_name the name of the action to check if preconditions are met in KB
         * @param params action grounded parameters
         * @param ground_dictionary return value gets written here by reference
         * @return true if action start is applicable, false otherwise
         */
        bool isActionStartApplicable(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary);

        /**
         * @brief overloaded function that checks if action start preconditions are consistent with internal KB information
         * @param action_name the name of the action to check if preconditions are met in KB
         * @param params action grounded parameters
         * @return true if action start is applicable, false otherwise
         */
        bool isActionStartApplicable(std::string &action_name, std::vector<std::string> &params);

        /**
         * @brief overloaded function that checks if action start preconditions are consistent with internal KB information
         * we dont't care here about the ground dictionary but we do care about the combined probability
         * @param action_name the name of the action to check if preconditions are met in KB
         * @param params action grounded parameters
         * @param combined_probability return value gets written here by reference, multiply the probability
         * of preconditions (action start and overall) being true
         * @return true if action start is applicable, false otherwise
         */
        bool isActionStartApplicable(std::string &action_name, std::vector<std::string> &params, double &combined_probability);

        /**
         * @brief overloaded function that checks if action end preconditions are consistent with internal KB information
         * and return by reference the ground dictionary for simulation action purposes
         * @param action_name the name of the action to check if preconditions are met in KB
         * @param params action grounded parameters
         * @param ground_dictionary return value gets written here by reference
         * @return true if action end is applicable, false otherwise
         */
        bool isActionEndApplicable(std::string &action_name, std::vector<std::string> &params,
            std::map<std::string, std::string> &ground_dictionary);

        /**
         * @brief overloaded function that checks if action end preconditions are consistent with internal KB information
         * and return by reference the ground dictionary for simulation action purposes
         * @param action_name the name of the action to check if preconditions are met in KB
         * @param params action grounded parameters
         * @return true if action end is applicable, false otherwise
         */
        bool isActionEndApplicable(std::string &action_name, std::vector<std::string> &params);

        /**
         * @brief overloaded function that checks if action end preconditions are consistent with internal KB information
         * and return by reference the ground dictionary for simulation action purposes
         * @param action_name the name of the action to check if preconditions are met in KB
         * @param params action grounded parameters
         * @param combined_probability return value gets written here by reference, multiply the probability
         * of preconditions (overall and end) being true
         * @return true if action end is applicable, false otherwise
         */
        bool isActionEndApplicable(std::string &action_name, std::vector<std::string> &params, double &combined_probability);

        /**
         * @brief helper function to avoid repetition of code, used to facilitate the creation of knowledge items
         * @param name the name of the fact
         * @param params fact arguments or parameters, e.g. (robot_at location1), name:robot_at, params:[location1]
         * @param is_negative if true then fact is not negated, e.g. (robot_at location1).
         * If false then fact is negated, e.g. (not (location_clean kitchen))
         * @return a fact encoded in the KnowledgeItem struct
         */
        rosplan_knowledge_msgs::KnowledgeItem createFactKnowledgeItem(std::string &name, std::vector<std::string> &params, bool is_negative);

        /**
         * @brief apply delete and add list to KB current state (start or end action, depends on parameter action_start)
         * @param action_name the name of the action to be simulated
         * @param params action parameters as list of strings
         * @param action_start input boolena: set to true for action start behavior, set false for action end
         * @return true if action was applied successful, false otherwise
         */
        bool simulateAction(std::string &action_name, std::vector<std::string> &params, bool action_start);

        /**
         * @brief apply delete and add list to KB current state (start action only)
         * @param action_name the name of the action to apply
         * @param params action parameters as list of strings
         * @return true if action was applied successful, false otherwise
         */
        bool simulateActionStart(std::string &action_name, std::vector<std::string> &params);

        /**
         * @brief apply delete and add list to KB current state (end action only)
         * @param action_name the name of the action to apply
         * @param params action parameters as list of strings
         * @return true if action was applied successful, false otherwise
         */
        bool simulateActionEnd(std::string &action_name, std::vector<std::string> &params);

        /**
         * @brief apply effects from Knowledge item array,
         * apply all of the effects in knowledge item array to a KB
         * @param effects
         * @return true if effects size is non 0, false otherwise
         */
        bool reverseEffectsFromKIA(std::vector<rosplan_knowledge_msgs::KnowledgeItem> effects);

        /**
         * @brief get from memory relevant effects that need to be reverted to the state
         * @param action_name fact name
         * @param params args for the fact
         * @param action_start true if action start, false if action end (temporal actions)
         */
        bool revertAction(std::string &action_name, std::vector<std::string> &params, bool action_start);

        /**
         * @brief this method uses the memory of simulateAction to revert exactly the effects that were added/removed
         * rather that checking from domain details
         * @param action_name the name of the action to revert
         * @param params the parameters of the action to revert
         * @return true if action was reverted succesfully, false otherwise
         */
        bool revertAction(std::string &action_name, std::vector<std::string> &params);

        /**
         * @brief inverse of apply action, used for backtracking purposes
         * revert means: check action effects and revert them (delete positive effects, add negative effects)
         * @param action_name the name of the action to revert
         * @param params the parameters of the action to revert
         * @param action_start if true, action start effects are reverted, if false action end params are reverted
         * @return true if action was reverted succesfully, false otherwise
         */
        bool revertActionBlind(std::string &action_name, std::vector<std::string> &params, bool action_start);

        /**
         * @brief overloaded function to revert action start effects
         * @param action_name the name of the action to revert
         * @param params the parameters of the action to revert
         * @return true if action was reverted succesfully, false otherwise
         */
        bool revertActionStart(std::string &action_name, std::vector<std::string> &params);

        /**
         * @brief overloaded function to revert action end effects
         * @param action_name the name of the action to revert
         * @param params the parameters of the action to revert
         * @return true if action was reverted succesfully, false otherwise
         */
        bool revertActionEnd(std::string &action_name, std::vector<std::string> &params);

        /**
         * @brief get all goals from real KB
         * @param kb_goals return value gets written in this variable by reference
         * @return true if call to real KB was possible, false otherwise
         */
        bool getAllGoals(std::vector<rosplan_knowledge_msgs::KnowledgeItem> &kb_goals);

        /**
         * @brief check if all goals are satisfied in internal KB
         * @return true if goals are satisfied (present in internal KB)
         */
        bool areGoalsAchieved();

        /**
         * @brief call real KB services and save them in member variables
         * this includes all operator names, operator details, etc. This funcion should only be called once
         * @param mirror_facts_and_goals if true, facts and goals are fetched from KB and stored
         * @return true if communication with real KB was successful, false otherwise
         */
        bool mirrorKB(bool mirror_facts_and_goals);

    private:

         /**
         * @brief initialize service clients for various KB services, they are stored in member variables for future use
         * this function gets called one time from constructor
         */
        void prepareServices();

        /**
         * @brief Check if service exists within a timeout of 10 secs
         * @param srv any ros::ServiceClient client already initialized
         * @return true if service exists, false otherwise
         */
        bool checkServiceExistance(ros::ServiceClient &srv);

        /// used to setup service connection clients with real KB
        ros::NodeHandle nh_;

        /// clients to communicate with real KB and get information from it (one time only)
        ros::ServiceClient od_srv_client_, on_srv_client_, dp_srv_client_, sp_srv_client_, sg_srv_client_;

        /// store a list of domain predicates
        std::vector<std::string> domain_predicates_;

        /// store a list of all domain operator names
        std::vector<std::string> operator_names_;

        /// store a list of ungrounded domain operator details
        std::vector<rosplan_knowledge_msgs::DomainOperator> domain_operator_details_;

        /// store a map of operator_names_ vs domain_operator_details_
        std::map<std::string, rosplan_knowledge_msgs::DomainOperator> domain_operator_map_;

        /// stores all grounded facts (is like an internal copy of KB)
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> kb_facts_;

        /// store a kb_facts_ bkp for reset internal KB purposes
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> kb_facts_bkp_;

        /// store all goals in KB
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> kb_goals_;

        /// to keep track of the simulated actions and be able to revert them accurately
        std::map<std::pair<std::string, std::vector<std::string> >, std::vector<rosplan_knowledge_msgs::KnowledgeItem> > start_sim_actions_map_;
        std::map<std::pair<std::string, std::vector<std::string> >, std::vector<rosplan_knowledge_msgs::KnowledgeItem> > end_sim_actions_map_;
};
