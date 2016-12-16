#include "rosplan_knowledge_base/KnowledgeBase.h"

namespace KCL_rosplan {

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

	/*------------------*/
	/* knowledge update */
	/*------------------*/

	bool KnowledgeBase::updateKnowledge(rosplan_knowledge_msgs::KnowledgeUpdateService::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateService::Response &res) {

		if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE)
			addKnowledge(req.knowledge);
		else if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL)
			addMissionGoal(req.knowledge);
		else if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE)
			removeKnowledge(req.knowledge);
		else if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL)
			removeMissionGoal(req.knowledge);

		res.success = true;
		return true;
	}

	bool KnowledgeBase::updateKnowledgeArray(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res) {

		if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE)
			for(int i=0;i<req.knowledge.size();i++) addKnowledge(req.knowledge[i]);
		else if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL)
			for(int i=0;i<req.knowledge.size();i++) addMissionGoal(req.knowledge[i]);
		else if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE)
			for(int i=0;i<req.knowledge.size();i++) removeKnowledge(req.knowledge[i]);
		else if(req.update_type == rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL)
			for(int i=0;i<req.knowledge.size();i++) removeMissionGoal(req.knowledge[i]);

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
			for(iit = model_instances[msg.instance_type].begin(); iit!=model_instances[msg.instance_type].end(); iit++) {

				std::string name = *iit;

				if(name.compare(msg.instance_name)==0 || msg.instance_name.compare("")==0) {

					// remove instance from knowledge base
					ROS_INFO("KCL: (KB) Removing instance (%s, %s)", msg.instance_type.c_str(), (msg.instance_name.compare("")==0) ? "ALL" : msg.instance_name.c_str());
					iit = model_instances[msg.instance_type].erase(iit);
					if(iit!=model_instances[msg.instance_type].begin()) iit--;
					plan_filter.checkFilters(msg, false);

					// remove affected domain attributes
					std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
					for(pit=model_facts.begin(); pit!=model_facts.end(); pit++) {
						if(KnowledgeComparitor::containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing domain attribute (%s)", pit->attribute_name.c_str());
							plan_filter.checkFilters(*pit, false);
							pit = model_facts.erase(pit);
							if(pit!=model_facts.begin()) pit--;
							if(pit==model_facts.end()) break;
						}
					}

					// finish
					if(iit==model_instances[msg.instance_type].end()) break;
				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION) {

			// remove domain attribute (function) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_functions.begin(); pit!=model_functions.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					plan_filter.checkFilters(msg, false);
					pit = model_functions.erase(pit);
					if(pit!=model_functions.begin()) pit--;
					if(pit==model_functions.end()) break;
				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

			// remove domain attribute (predicate) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_facts.begin(); pit!=model_facts.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					plan_filter.checkFilters(msg, false);
					pit = model_facts.erase(pit);
					if(pit!=model_facts.begin()) pit--;
					if(pit==model_facts.end()) break;
				}
			}
		}
	}

	/**
	 * remove everything
	 */
	bool KnowledgeBase::clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

		ROS_INFO("KCL: (KB) Removing whole model");

		// model
		model_instances.clear();
		model_facts.clear();
		model_functions.clear();
		model_goals.clear();
	}

	/**
	 * remove mission goal
	 */
	void KnowledgeBase::removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		bool changed = false;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator git;
		for(git=model_goals.begin(); git!=model_goals.end(); git++) {
			if(KnowledgeComparitor::containsKnowledge(msg, *git)) {
				ROS_INFO("KCL: (KB) Removing goal (%s)", msg.attribute_name.c_str());
				git = model_goals.erase(git);
				if(git!=model_goals.begin()) git--;
				if(git==model_goals.end()) break;
			}
		}

		if(changed) {			
			rosplan_knowledge_msgs::Notification notMsg;
			notMsg.function = rosplan_knowledge_msgs::Notification::REMOVED;
			notMsg.knowledge_item = msg;
			plan_filter.notification_publisher.publish(notMsg);
		}
	}

	/*--------------*/
	/* adding items */
	/*--------------*/

	/*
	 * add an instance, domain predicate, or function to the knowledge base
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
				plan_filter.checkFilters(msg, true);
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

			// add domain attribute
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_facts.begin(); pit!=model_facts.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					return;
				}
			}
			ROS_INFO("KCL: (KB) Adding domain attribute (%s)", msg.attribute_name.c_str());
			model_facts.push_back(msg);
			plan_filter.checkFilters(msg, true);

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
			plan_filter.checkFilters(msg, true);
		}
	}

	/*
	 * add mission goal to knowledge base
	 */
	void KnowledgeBase::addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
		for(pit=model_goals.begin(); pit!=model_goals.end(); pit++) {
			if(KnowledgeComparitor::containsKnowledge(msg, *pit))
				return;
		}
		ROS_INFO("KCL: (KB) Adding mission goal (%s)", msg.attribute_name.c_str());
		model_goals.push_back(msg);
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

		// fetch the knowledgeItems of the correct attribute
		for(size_t i=0; i<model_facts.size(); i++) {
			if(0==req.predicate_name.compare(model_facts[i].attribute_name) || ""==req.predicate_name)
				res.attributes.push_back(model_facts[i]);
		}

		// ...or fetch the knowledgeItems of the correct function
		for(size_t i=0; i<model_functions.size(); i++) {
			if(0==req.predicate_name.compare(model_functions[i].attribute_name) || ""==req.predicate_name)
				res.attributes.push_back(model_functions[i]);
		}

		return true;
	}

	bool KnowledgeBase::getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {

		for(size_t i=0; i<model_goals.size(); i++)
			res.attributes.push_back(model_goals[i]);
		return true;
	}

	/*-----------------*/
	/* fetching domain */
	/*-----------------*/

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

} // close namespace

/*-------------*/
/* main method */
/*-------------*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_knowledge_base");
	ros::NodeHandle n;

	// parameters
	std::string domainPath;
	n.param("/rosplan/domain_path", domainPath, std::string("common/domain.pddl"));

	KCL_rosplan::KnowledgeBase kb;
	ROS_INFO("KCL: (KB) Parsing domain");
	kb.domain_parser.domain_parsed = false;
	kb.domain_parser.parseDomain(domainPath);

	// fetch domain info
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
	ros::ServiceServer clearServer = n.advertiseService("/kcl_rosplan/clear_knowledge_base", &KCL_rosplan::KnowledgeBase::clearKnowledge, &kb);

	// fetch knowledge
	ros::ServiceServer currentInstanceServer = n.advertiseService("/kcl_rosplan/get_current_instances", &KCL_rosplan::KnowledgeBase::getCurrentInstances, &kb);
	ros::ServiceServer currentKnowledgeServer = n.advertiseService("/kcl_rosplan/get_current_knowledge", &KCL_rosplan::KnowledgeBase::getCurrentKnowledge, &kb);
	ros::ServiceServer currentGoalServer = n.advertiseService("/kcl_rosplan/get_current_goals", &KCL_rosplan::KnowledgeBase::getCurrentGoals, &kb);

	// planning and mission filter
	kb.plan_filter.notification_publisher = n.advertise<rosplan_knowledge_msgs::Notification>("/kcl_rosplan/notification", 10, true);
	ros::Subscriber planningFilterSub = n.subscribe("/kcl_rosplan/plan_filter", 100, &KCL_rosplan::PlanFilter::planningFilterCallback, &kb.plan_filter);
	ros::Subscriber missionFilterSub = n.subscribe("/kcl_rosplan/plan_filter", 100, &KCL_rosplan::PlanFilter::missionFilterCallback, &kb.plan_filter);

	// wait for and clear mongoDB 
	ROS_INFO("KCL: (KB) Waiting for MongoDB");
	ros::service::waitForService("/message_store/delete",-1);
	system("mongo message_store --eval \"printjson(db.message_store.remove())\"");

	ROS_INFO("KCL: (KB) Ready to receive");
	ros::spin();

	return 0;
}

