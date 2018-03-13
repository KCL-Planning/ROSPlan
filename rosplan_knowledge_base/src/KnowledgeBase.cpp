#include "rosplan_knowledge_base/KnowledgeBase.h"

namespace KCL_rosplan {

	void KnowledgeBase::runKnowledgeBase() {

		while(ros::ok()) {

			// update TILs
			std::map<ros::Time, rosplan_knowledge_msgs::KnowledgeItem>::iterator tit = model_timed_initial_literals.begin();
			for(; tit != model_timed_initial_literals.end(); ) {
				if(tit->first <= ros::Time::now()) {
					if(tit->second.is_negative) {
						tit->second.is_negative = false;
						removeKnowledge(tit->second);
					} else {
						addKnowledge(tit->second);
					}
					model_timed_initial_literals.erase(tit++);
				} else {
					tit++;
				}
			}

			// services
			ros::spinOnce();
		}
	}

	/*-----------------*/
	/* knowledge query */
	/*-----------------*/

	bool KnowledgeBase::queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res) {

		res.all_true = true;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
		for(iit = req.knowledge.begin(); iit!=req.knowledge.end(); iit++) {

			bool present = false;
			if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {

				// check if instance exists
				std::vector<std::string>::iterator sit;
				sit = find(model_instances[iit->instance_type].begin(), model_instances[iit->instance_type].end(), iit->instance_name);
				present = (sit!=model_instances[iit->instance_type].end());
				
			} else if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION) {

				// check if function exists; TODO inequalities
				std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
				for(pit=model_functions.begin(); pit!=model_functions.end(); pit++) {
					if(KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
						present = true;
						pit = model_functions.end();
					}
				}
			} else if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

				// check if fact is true
				std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
				for(pit=model_facts.begin(); pit!=model_facts.end(); pit++) {
					if(KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
						present = true;
						break;
					}
				}
			}

			if(!present) {
				res.all_true = false;
				res.results.push_back(false);
				res.false_knowledge.push_back(*iit);
			} else {
				res.results.push_back(true);
			}
		}

		return true;
	}

	/*--------------------*/
	/* adding constraints */
	/*--------------------*/

	bool KnowledgeBase::updateKnowledgeConstraintsOneOf(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res) {

		int count = 0;
		for(int i=0;i<req.knowledge.size();i++) {
			// check if fact is true
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_facts.begin(); pit!=model_facts.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(req.knowledge[i], *pit)) {
					count++;
				}
			}
		}
		if(count>1)	ROS_ERROR("KCL: (KB) Warning: more than one Knowledge Item is true in new OneOf constraint!");
		model_oneof_constraints.push_back(req.knowledge);
		res.success = true;
		return true;
	}

	/*------------------*/
	/* knowledge update */
	/*------------------*/

	bool KnowledgeBase::updateKnowledge(rosplan_knowledge_msgs::KnowledgeUpdateService::Request &req, rosplan_knowledge_msgs::KnowledgeUpdateService::Response &res) {

		switch(req.update_type) {
		case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE:
			if(req.duration>0) {
				ros::Time time = ros::Time::now() + ros::Duration(req.duration);
				model_timed_initial_literals.insert(std::make_pair(time,req.knowledge));
			} else {
				addKnowledge(req.knowledge);
			}
			break;
		case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL:
			addMissionGoal(req.knowledge);
			break;
		case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE:
			if(req.duration>0) {
				ros::Time time = ros::Time::now() + ros::Duration(req.duration);
				req.knowledge.is_negative = !req.knowledge.is_negative;
				model_timed_initial_literals.insert(std::make_pair(time,req.knowledge));
			} else {
				removeKnowledge(req.knowledge);
			}
			break;
		case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL:
			removeMissionGoal(req.knowledge);
			break;
		
		case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_METRIC:
            		addMissionMetric(req.knowledge);
			break;
         	case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_METRIC:
            		removeMissionMetric(req.knowledge);
			break;
        	}

		res.success = true;
		return true;
	}

	bool KnowledgeBase::updateKnowledgeArray(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res) {

		switch(req.update_type) {
		case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE:
			for(int i=0;i<req.knowledge.size();i++) {
				if(req.duration>0) {
					ros::Time time = ros::Time::now() + ros::Duration(req.duration);
					model_timed_initial_literals.insert(std::make_pair(time,req.knowledge[i]));
				} else {
					addKnowledge(req.knowledge[i]);
				}
			}
			break;
		case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL:
			for(int i=0;i<req.knowledge.size();i++) {
				addMissionGoal(req.knowledge[i]);
			}
			break;
		case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE:
			for(int i=0;i<req.knowledge.size();i++) {
				if(req.duration>0) {
					ros::Time time = ros::Time::now() + ros::Duration(req.duration);
					req.knowledge[i].is_negative = !req.knowledge[i].is_negative;
					model_timed_initial_literals.insert(std::make_pair(time,req.knowledge[i]));
				} else {
					removeKnowledge(req.knowledge[i]);
				}
			}
			break;
		case rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL:
			for(int i=0;i<req.knowledge.size();i++) {
				removeMissionGoal(req.knowledge[i]);
			}
			break;
		}

		res.success = true;
		return true;
	}

	/*----------------*/
	/* removing items */
	/*----------------*/

	void KnowledgeBase::removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {		

			// search for instance
			std::vector<std::string>::iterator iit;
			for(iit = model_instances[msg.instance_type].begin(); iit!=model_instances[msg.instance_type].end(); ) {

				std::string name = *iit;

				if(name.compare(msg.instance_name)==0 || msg.instance_name.compare("")==0) {

					// remove instance from knowledge base
					ROS_INFO("KCL: (KB) Removing instance (%s, %s)", msg.instance_type.c_str(), (msg.instance_name.compare("")==0) ? "ALL" : msg.instance_name.c_str());
					iit = model_instances[msg.instance_type].erase(iit);
					if(iit!=model_instances[msg.instance_type].begin()) iit--;

					// remove affected domain attributes
					std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
					for(pit=model_facts.begin(); pit!=model_facts.end(); ) {
						if(KnowledgeComparitor::containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing domain attribute (%s)", pit->attribute_name.c_str());
							pit = model_facts.erase(pit);
						} else {
							pit++;
						}
					}
				} else {

					iit++;

				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION) {

			// remove domain attribute (function) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_functions.begin(); pit!=model_functions.end(); ) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					pit = model_functions.erase(pit);
				} else {
					pit++;
				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

			// remove domain attribute (predicate) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_facts.begin(); pit!=model_facts.end(); ) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					pit = model_facts.erase(pit);
				} else {
					pit++;
				}
			}
		}
	}

	/**
	 * remove everything
	 */
	bool KnowledgeBase::clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

        rosplan_knowledge_msgs::KnowledgeItem empty;

		ROS_INFO("KCL: (KB) Removing whole model");

		// model
		model_instances.clear();
		model_facts.clear();
		model_functions.clear();
		model_goals.clear();
        model_metric = empty;
	}

	/**
	 * remove mission goal
	 */
	void KnowledgeBase::removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		bool changed = false;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator git;
		for(git=model_goals.begin(); git!=model_goals.end(); ) {
			if(KnowledgeComparitor::containsKnowledge(msg, *git)) {
				ROS_INFO("KCL: (KB) Removing goal (%s)", git->attribute_name.c_str());
				git = model_goals.erase(git);
			} else {
				git++;
			}
		}
	}

    /**
	 * remove mission metric
	 */
    void KnowledgeBase::removeMissionMetric(rosplan_knowledge_msgs::KnowledgeItem &msg) {

        rosplan_knowledge_msgs::KnowledgeItem empty;
        ROS_INFO("KCL: (KB) Removing metric");
                model_metric = empty;
    }

	/*--------------*/
	/* adding items */
	/*--------------*/

	/*
	 * add an instance, fact, or function to the knowledge base
	 */
	void KnowledgeBase::addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg) {
		
		if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {

			// check if instance is already in knowledge base
			std::vector<std::string>::iterator iit;
			iit = find(model_instances[msg.instance_type].begin(), model_instances[msg.instance_type].end(), msg.instance_name);

			// add instance
			if(iit==model_instances[msg.instance_type].end()) {
				ROS_INFO("KCL: (KB) Adding instance (%s, %s)", msg.instance_type.c_str(), msg.instance_name.c_str());
				model_instances[msg.instance_type].push_back(msg.instance_name);
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

			// add domain attribute
			std::string param_str;
			for (size_t i = 0; i < msg.values.size(); ++i) {
				param_str += " " + msg.values[i].key + "=" + msg.values[i].value;
			}

			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_facts.begin(); pit!=model_facts.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					ROS_WARN("KCL: (KB) Domain attribute (%s%s) already exists",
					msg.attribute_name.c_str(), param_str.c_str());
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding domain attribute (%s%s)",
			msg.attribute_name.c_str(), param_str.c_str());
			model_facts.push_back(msg);

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION) {

			// add domain function
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_functions.begin(); pit!=model_functions.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					// already added TODO value check
					ROS_INFO("KCL: (KB) Updating domain function (%s)", msg.attribute_name.c_str());
					pit->function_value = msg.function_value;
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding domain function (%s)", msg.attribute_name.c_str());
			model_functions.push_back(msg);
		}
	}

	/*
	 * add mission goal to knowledge base
	 */
	void KnowledgeBase::addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		std::string param_str;
		for (size_t i = 0; i < msg.values.size(); ++i) {
			param_str += " " + msg.values[i].key + "=" + msg.values[i].value;
		}

		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
		for(pit=model_goals.begin(); pit!=model_goals.end(); pit++) {
			if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
				ROS_WARN("KCL: (KB) Goal (%s%s) already posted",
				         msg.attribute_name.c_str(), param_str.c_str());
				return;
			}
		}
		
		ROS_INFO("KCL: (KB) Adding mission goal (%s%s)",
		         msg.attribute_name.c_str(), param_str.c_str());
		model_goals.push_back(msg);
	}

    /*
	 * add mission metric to knowledge base
	 */
    void KnowledgeBase::addMissionMetric(rosplan_knowledge_msgs::KnowledgeItem &msg) {

        ROS_INFO("KCL: (KB) Adding mission metric");
        model_metric = msg;
    }

	/*----------------*/
	/* fetching items */
	/*----------------*/

	bool KnowledgeBase::getCurrentInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res) {
	
		// fetch the instances of the correct type
		if(""==req.type_name) {
			std::map<std::string,std::vector<std::string> >::iterator iit;
			for(iit=model_instances.begin(); iit != model_instances.end(); iit++) {
				for(size_t j=0; j<iit->second.size(); j++)
					res.instances.push_back(iit->second[j]);
			}
		} else {
			std::map<std::string,std::vector<std::string> >::iterator iit;
			iit = model_instances.find(req.type_name);
			if(iit != model_instances.end()) {
				for(size_t j=0; j<iit->second.size(); j++)
					res.instances.push_back(iit->second[j]);
			}
		}

		return true;
	}

	bool KnowledgeBase::getCurrentKnowledge(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {

		ros::Time time = ros::Time::now();

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<model_facts.size(); i++) {
			if(0==req.predicate_name.compare(model_facts[i].attribute_name) || ""==req.predicate_name) {
				res.attributes.push_back(model_facts[i]);
				res.initial_time.push_back(time);
			}
		}

		// ...or fetch the knowledgeItems of the correct function
		for(size_t i=0; i<model_functions.size(); i++) {
			if(0==req.predicate_name.compare(model_functions[i].attribute_name) || ""==req.predicate_name) {
				res.attributes.push_back(model_functions[i]);
				res.initial_time.push_back(time);
			}
		}

		// ...and also all TILs and TIFs
		std::map<ros::Time, rosplan_knowledge_msgs::KnowledgeItem>::iterator tit = model_timed_initial_literals.begin();
		for(; tit != model_timed_initial_literals.end(); tit++) {
			if(0==req.predicate_name.compare(tit->second.attribute_name) || ""==req.predicate_name) {
				res.attributes.push_back(tit->second);
				res.initial_time.push_back(tit->first);
			}
		}

		return true;
	}

	bool KnowledgeBase::getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {

		for(size_t i=0; i<model_goals.size(); i++)
			res.attributes.push_back(model_goals[i]);
		return true;
	}

    bool KnowledgeBase::getCurrentMetric(rosplan_knowledge_msgs::GetMetricService::Request  &req, rosplan_knowledge_msgs::GetMetricService::Response &res) {
            res.metric = model_metric;
        return true;
    }

	/*-----------------*/
	/* fetching domain */
	/*-----------------*/

	/* get domain name */
	bool KnowledgeBase::getDomainName(rosplan_knowledge_msgs::GetDomainNameService::Request  &req, rosplan_knowledge_msgs::GetDomainNameService::Response &res) {
		if(!domain_parser.domain_parsed) return false;
		res.domain_name = domain_parser.domain->name;
		return true;
	}

	/* get domain types */
	bool KnowledgeBase::getTypes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res) {

		if(!domain_parser.domain_parsed) return false;

		VAL::pddl_type_list* types = domain_parser.domain->types;
		for (VAL::pddl_type_list::const_iterator ci = types->begin(); ci != types->end(); ci++) {
			const VAL::pddl_type* type = *ci;
			res.types.push_back(type->getName());
			if(type->type) res.super_types.push_back(type->type->getName());
			else res.super_types.push_back("");
		}	
		return true;
	}		

	/* get domain predicates */
	bool KnowledgeBase::getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {

		VAL::pred_decl_list* predicates = domain_parser.domain->predicates;
		if(predicates) {
			for (VAL::pred_decl_list::const_iterator ci = predicates->begin(); ci != predicates->end(); ci++) {
				const VAL::pred_decl* predicate = *ci;

				// predicate name
				rosplan_knowledge_msgs::DomainFormula formula;
				formula.name = predicate->getPred()->symbol::getName();

				// predicate variables
				for (VAL::var_symbol_list::const_iterator vi = predicate->getArgs()->begin(); vi != predicate->getArgs()->end(); vi++) {
					const VAL::var_symbol* var = *vi;
					diagnostic_msgs::KeyValue param;
					param.key = var->pddl_typed_symbol::getName();
					param.value = var->type->getName();
					formula.typed_parameters.push_back(param);
				}
				res.items.push_back(formula);
			}
		}
		return true;
	}

	/* get domain functions */
	bool KnowledgeBase::getFunctions(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res) {

		VAL::func_decl_list* functions = domain_parser.domain->functions;
		if(functions) {
			for (VAL::func_decl_list::const_iterator ci = functions->begin(); ci != functions->end(); ci++) {
				const VAL::func_decl* function = *ci;

				// function name
				rosplan_knowledge_msgs::DomainFormula formula;
				formula.name = function->getFunction()->symbol::getName();

				// parameters
				for (VAL::var_symbol_list::const_iterator vi = function->getArgs()->begin(); vi != function->getArgs()->end(); vi++) {
					const VAL::var_symbol* var = *vi;
					diagnostic_msgs::KeyValue param;
					param.key = var->pddl_typed_symbol::getName();
					param.value = var->type->getName();
					formula.typed_parameters.push_back(param);
				}
				res.items.push_back(formula);
			}
		}
		return true;
	}

	/* get domain operators */
	bool KnowledgeBase::getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res) {

		VAL::operator_list* operators = domain_parser.domain->ops;
		for (VAL::operator_list::const_iterator ci = operators->begin(); ci != operators->end(); ci++) {			
			const VAL::operator_* op = *ci;

			// name
			rosplan_knowledge_msgs::DomainFormula formula;
			formula.name = op->name->symbol::getName();

			// parameters
			for (VAL::var_symbol_list::const_iterator vi = op->parameters->begin(); vi != op->parameters->end(); vi++) {
				const VAL::var_symbol* var = *vi;
				diagnostic_msgs::KeyValue param;
				param.key = var->pddl_typed_symbol::getName();
				param.value = var->type->getName();
				formula.typed_parameters.push_back(param);
			}

			res.operators.push_back(formula);
		}
		return true;
	}

	/* get domain operator details */
	bool KnowledgeBase::getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res) {
		VALVisitorOperator op_visitor;
		VAL::operator_list* operators = domain_parser.domain->ops;
		for (VAL::operator_list::const_iterator ci = operators->begin(); ci != operators->end(); ci++) {			
			if((*ci)->name->symbol::getName() == req.name) {
				op_visitor.visit_operator_(*ci);
				res.op = op_visitor.msg;
				return true;
			}
		}
		return false;
	}

	/* get domain predicate details */
	bool KnowledgeBase::getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res) {
		VALVisitorPredicate pred_visitor;
		VAL::pred_decl_list* predicates = domain_parser.domain->predicates;
		for (VAL::pred_decl_list::const_iterator ci = predicates->begin(); ci != predicates->end(); ci++) {			
			if((*ci)->getPred()->symbol::getName() == req.name) {
				pred_visitor.visit_pred_decl(*ci);
				res.predicate = pred_visitor.msg;
				return true;
			}
		}
		return false;
	}

    /*------------------------------------*/
    /* add initial state to knowledge base*/
    /*------------------------------------*/

    /* get the initial state from the domain and problem files*/
    void KnowledgeBase::addInitialState(VAL::domain* domain, VAL::problem* problem) {

        VALVisitorProblem problem_visitor(domain,problem);
        model_instances = problem_visitor.returnInstances();
        model_facts = problem_visitor.returnFacts();
        model_goals = problem_visitor.returnGoals();
        model_metric = problem_visitor.returnMetric();

    }

} // close namespace

/*-------------*/
/* main method */
/*-------------*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_knowledge_base");
	ros::NodeHandle n("~");

    // parameters
    std::string domainPath, problemPath;
    bool useUnknowns;
    n.param("/rosplan/domain_path", domainPath, std::string("common/domain.pddl"));
    n.param("use_unknowns", useUnknowns, false);
    n.param("domain_path", domainPath, domainPath);
    n.param("problem_path", problemPath, problemPath);


    KCL_rosplan::KnowledgeBase kb;
	std::ifstream file_check;

    // parse domain
    ROS_INFO("KCL: (KB) Parsing domain");
    kb.domain_parser.domain_parsed = false;
	VAL::domain* domain;
	file_check.open(domainPath.c_str());
	if(!file_check.good()) {
	    ROS_ERROR("KCL: (KB) Domain file does not exist.");
		return 0;
	} else {
		file_check.close();
	    domain = kb.domain_parser.parseDomain(domainPath);
	}

    // parse problem and add initial state
	if(problemPath != "") {
		ROS_INFO("KCL: (KB) Parsing initial state");
		kb.problem_parser.problem_parsed = false;
		file_check.open(problemPath.c_str());
		if(!file_check.good()) {
			ROS_WARN("KCL: (KB) Initial state file does not exist.");
		} else {
			file_check.close();
			VAL::problem* problem = kb.problem_parser.parseProblem(problemPath);
			kb.addInitialState(domain, problem);
		}
	}

    kb.use_unknowns = useUnknowns;

	// fetch domain info
	ros::ServiceServer nameServer = n.advertiseService("/kcl_rosplan/get_domain_name", &KCL_rosplan::KnowledgeBase::getDomainName, &kb);
	ros::ServiceServer typeServer = n.advertiseService("/kcl_rosplan/get_domain_types", &KCL_rosplan::KnowledgeBase::getTypes, &kb);
	ros::ServiceServer predicateServer = n.advertiseService("/kcl_rosplan/get_domain_predicates", &KCL_rosplan::KnowledgeBase::getPredicates, &kb);
	ros::ServiceServer functionServer = n.advertiseService("/kcl_rosplan/get_domain_functions", &KCL_rosplan::KnowledgeBase::getFunctions, &kb);
	ros::ServiceServer operatorServer = n.advertiseService("/kcl_rosplan/get_domain_operators", &KCL_rosplan::KnowledgeBase::getOperators, &kb);

	ros::ServiceServer opDetailsServer = n.advertiseService("/kcl_rosplan/get_domain_operator_details", &KCL_rosplan::KnowledgeBase::getOperatorDetails, &kb);
	ros::ServiceServer predDetailsServer = n.advertiseService("/kcl_rosplan/get_domain_predicate_details", &KCL_rosplan::KnowledgeBase::getPredicateDetails, &kb);

	// query knowledge
	ros::ServiceServer queryServer = n.advertiseService("/kcl_rosplan/query_knowledge_base", &KCL_rosplan::KnowledgeBase::queryKnowledge, &kb);

	// update knowledge
	ros::ServiceServer updateServer1 = n.advertiseService("/kcl_rosplan/update_knowledge_base", &KCL_rosplan::KnowledgeBase::updateKnowledge, &kb);
	ros::ServiceServer updateServer2 = n.advertiseService("/kcl_rosplan/update_knowledge_base_array", &KCL_rosplan::KnowledgeBase::updateKnowledgeArray, &kb);
	ros::ServiceServer updateServer3 = n.advertiseService("/kcl_rosplan/update_knowledge_base_constraints_oneof", &KCL_rosplan::KnowledgeBase::updateKnowledgeConstraintsOneOf, &kb);
	ros::ServiceServer clearServer = n.advertiseService("/kcl_rosplan/clear_knowledge_base", &KCL_rosplan::KnowledgeBase::clearKnowledge, &kb);

	// fetch knowledge
	ros::ServiceServer currentInstanceServer = n.advertiseService("/kcl_rosplan/get_current_instances", &KCL_rosplan::KnowledgeBase::getCurrentInstances, &kb);
	ros::ServiceServer currentKnowledgeServer = n.advertiseService("/kcl_rosplan/get_current_knowledge", &KCL_rosplan::KnowledgeBase::getCurrentKnowledge, &kb);
	ros::ServiceServer currentGoalServer = n.advertiseService("/kcl_rosplan/get_current_goals", &KCL_rosplan::KnowledgeBase::getCurrentGoals, &kb);
    	ros::ServiceServer currentMetricServer = n.advertiseService("/kcl_rosplan/get_current_metric", &KCL_rosplan::KnowledgeBase::getCurrentMetric, &kb);

	// wait for and clear mongoDB 
	ROS_INFO("KCL: (KB) Waiting for MongoDB");
	ros::service::waitForService("/message_store/delete",-1);
	system("mongo message_store --eval \"printjson(db.message_store.remove())\"");

	ROS_INFO("KCL: (KB) Ready to receive");
	kb.runKnowledgeBase();

	return 0;
}

