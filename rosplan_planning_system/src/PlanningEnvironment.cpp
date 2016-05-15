#include "rosplan_planning_system/PlanningEnvironment.h"

/* implementation of rosplan_planning_system::PlanningEnvironment */
namespace KCL_rosplan {

	std::string toLowerCase(const std::string& str) {
		std::stringstream ss;	
		int differ = 'A'-'a';
		int ii = str.size();
		for(int i=0; i<ii;i++) {
			char ch = str.at(i);
			if(ch>='A' && ch<='Z')
				ch = ch-differ;
			ss << ch;
		}
		return ss.str();
	}

	/*----------------*/
	/* parsing domain */
	/*----------------*/

	/**
	 * read domain: used for writing the PDDL problem file and then for constructing the dispatch
	 * messages.
	 */
	void PlanningEnvironment::parseDomain(const std::string domainPath) {
		
		// only parse domain once
		if(domain_parsed)
			return;

		domain_parsed = true;
		std::string domainFileName = (domainPath);
		ROS_INFO("KCL: (PS) Parsing domain: %s.", domainFileName.c_str());

		
		// save filename for VAL
		std::vector<char> writable(domainFileName.begin(), domainFileName.end());
		writable.push_back('\0');
		current_filename = &writable[0];		
		
		VAL::current_analysis = &VAL::an_analysis;
		std::ifstream domainFile;
		domainFile.open(domainFileName.c_str());
		yydebug = 0;
		VAL::yfl = new yyFlexLexer;

		if (!domainFile.is_open() || domainFile.fail() || domainFile.bad()) {
			ROS_ERROR("KCL: (PS) Failed to open domain file.");
			line_no = 0;
			VAL::log_error(VAL::E_FATAL,"Failed to open file");
		} else {
			
			ROS_INFO("KCL: (PS) File does exist: %s.", domainFileName.c_str());

			
			line_no = 1;
			VAL::yfl->switch_streams(&domainFile,&std::cout);
			yyparse();
			
			// domain name
			VAL::domain* domain = VAL::current_analysis->the_domain;
			if (domain == NULL)
			{
				ROS_ERROR("KCL: (PS) Domain could not be parsed!");
				return;
			}
			
			domainName = domain->name;
			
			// types
			VAL::pddl_type_list* types = domain->types;
			for (VAL::pddl_type_list::const_iterator ci = types->begin(); ci != types->end(); ci++) {
				const VAL::pddl_type* type = *ci;
				domain_types.push_back(type->getName());
			}
			
			// predicates
			VAL::pred_decl_list* predicates = domain->predicates;
			if(predicates) {
				for (VAL::pred_decl_list::const_iterator ci = predicates->begin(); ci != predicates->end(); ci++) {
					const VAL::pred_decl* predicate = *ci;
					// predicate name
					domain_predicates[predicate->getPred()->symbol::getName()];
					// parameters
					for (VAL::var_symbol_list::const_iterator vi = predicate->getArgs()->begin(); vi != predicate->getArgs()->end(); vi++) {
						const VAL::var_symbol* var = *vi;
						domain_predicates[predicate->getPred()->symbol::getName()].push_back(var->pddl_typed_symbol::getName());
					}
				}
			}
			
			// functions
			VAL::func_decl_list* functions = domain->functions;
			if(functions) {
				for (VAL::func_decl_list::const_iterator ci = functions->begin(); ci != functions->end(); ci++) {
					const VAL::func_decl* function = *ci;
					// function name
					domain_functions[function->getFunction()->symbol::getName()];
					// parameters
					for (VAL::var_symbol_list::const_iterator vi = function->getArgs()->begin(); vi != function->getArgs()->end(); vi++) {
						const VAL::var_symbol* var = *vi;
						domain_functions[function->getFunction()->symbol::getName()].push_back(var->pddl_typed_symbol::getName());
					}
				}
			}
			
			// operators
			VAL::operator_list* operators = domain->ops;
			for (VAL::operator_list::const_iterator ci = operators->begin(); ci != operators->end(); ci++) {			
				const VAL::operator_* op = *ci;
				// operator name
				domain_operators[op->name->symbol::getName()];
				// parameters
				for (VAL::var_symbol_list::const_iterator vi = op->parameters->begin(); vi != op->parameters->end(); vi++) {
					const VAL::var_symbol* var = *vi;
					domain_operators[op->name->symbol::getName()].push_back(var->pddl_typed_symbol::getName());
					domain_operator_types[op->name->symbol::getName()].push_back(var->pddl_typed_symbol::type->getName());
				}
				// preconditions
				const VAL::goal* precondition = op->precondition;
				domain_operator_precondition_map[op->name->symbol::getName()];
				parsePrecondition(op->name->symbol::getName(), precondition, false);
			}
		}
		domainFile.close();
		delete VAL::yfl;
		
		ROS_INFO("KCL: (PS) Finished parsing domain: %s.", domainFileName.c_str());
	}

	/**
	 * parse a precondition recursively.
	 * The final simple preconditions are stored in domain_operator_precondition_map.
	 * TODO: test negative precondition support.
	 */
	void PlanningEnvironment::parsePrecondition(const std::string &opName, const VAL::goal* goal, bool negative) {
		
		const VAL::neg_goal* ng = dynamic_cast<const VAL::neg_goal*>(goal);
		if (ng) {
			parsePrecondition(opName, ng->getGoal(), !negative);
			return;
		}

		const VAL::simple_goal* sg = dynamic_cast<const VAL::simple_goal*>(goal);
		if (sg) {
			const VAL::proposition* prop = sg->getProp();
			std::vector<string> precondition;
			precondition.push_back(prop->head->symbol::getName());
	        for (VAL::parameter_symbol_list::const_iterator ci = prop->args->begin(); ci != prop->args->end(); ci++) {
				const VAL::parameter_symbol* param = *ci;
				precondition.push_back(param->symbol::getName());
			}
			domain_operator_precondition_map[opName].push_back(precondition);
			return;
		}

		const VAL::timed_goal* tg = dynamic_cast<const VAL::timed_goal*>(goal);
		if (tg) {
			parsePrecondition(opName, tg->getGoal(), negative);
			return;
		}

		const VAL::conj_goal* cg = dynamic_cast<const VAL::conj_goal*>(goal);
		if (cg) {
	        const VAL::goal_list* goals = cg->getGoals();
	        for (VAL::goal_list::const_iterator ci = goals->begin(); ci != goals->end(); ci++) {
                parsePrecondition(opName, (*ci), negative);
			}
			return;
		}
	}

	/*----------------------*/
	/* updating environment */
	/*----------------------*/

	void PlanningEnvironment::clear() {

		// clear old problem
		type_object_map.clear();
		object_type_map.clear();
		instance_attributes.clear();
		domain_attributes.clear();
		goal_attributes.clear();
	}

	/**
	 * requests all the information required to build a problem instance.
	 */
	void PlanningEnvironment::update(ros::NodeHandle nh) {

		ROS_INFO("KCL: (PS) Fetching objects");

		// clear old problem
		clear();

		// setup service calls
		ros::ServiceClient GetInstancesClient = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		ros::ServiceClient GetDomainAttrsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		ros::ServiceClient GetCurrentGoalsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_goals");

		// for each type fetch instances
		for(size_t t=0; t<domain_types.size(); t++) {

			rosplan_knowledge_msgs::GetInstanceService instanceSrv;
			instanceSrv.request.type_name = domain_types[t];
			type_object_map[domain_types[t]];

			if (GetInstancesClient.call(instanceSrv)) {
				for(size_t i=0;i<instanceSrv.response.instances.size();i++) {

					// add new instance (popf converts names to lowercase)
					std::string name = instanceSrv.response.instances[i];
					name_map[KCL_rosplan::toLowerCase(name)] = name;
					type_object_map[domain_types[t]].push_back(name);
					object_type_map[name] = domain_types[t];
				}
			} else {
				ROS_ERROR("KCL: (PS) Failed to call service /kcl_rosplan/get_instances: %s", instanceSrv.request.type_name.c_str());
			}
		}
		ROS_INFO("KCL: (PS) Fetch attributes and functions");

		// get domain attributes and functions
		std::map<std::string,std::vector<std::string> >::iterator ait;
		for(ait = domain_predicates.begin(); ait != domain_predicates.end(); ait++) {
			rosplan_knowledge_msgs::GetAttributeService domainAttrSrv;
			domainAttrSrv.request.predicate_name = ait->first;
			
			if (GetDomainAttrsClient.call(domainAttrSrv)) {
				for(size_t j=0;j<domainAttrSrv.response.attributes.size();j++) {
					rosplan_knowledge_msgs::KnowledgeItem attr = domainAttrSrv.response.attributes[j];
					if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT && attr.attribute_name.compare(ait->first)==0)
					{
						domain_attributes.push_back(attr);
					}
				}
			} else {
				ROS_ERROR("KCL: (PS) Failed to call service /kcl_rosplan/get_domain_attributes %s", domainAttrSrv.request.predicate_name.c_str());
			}
		}
		for(ait = domain_functions.begin(); ait != domain_functions.end(); ait++) {
			rosplan_knowledge_msgs::GetAttributeService domainAttrSrv;
			domainAttrSrv.request.predicate_name = ait->first;
			if (GetDomainAttrsClient.call(domainAttrSrv)) {
				for(size_t j=0;j<domainAttrSrv.response.attributes.size();j++) {
					rosplan_knowledge_msgs::KnowledgeItem attr = domainAttrSrv.response.attributes[j];
					if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION && attr.attribute_name.compare(ait->first)==0)
						domain_attributes.push_back(attr);
				}
			} else {
				ROS_ERROR("KCL: (PS) Failed to call service /kcl_rosplan/get_domain_attributes %s", domainAttrSrv.request.predicate_name.c_str());
			}
		}

		// get current goals
		ROS_INFO("KCL: (PS) Fetch goals");
		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		if (GetCurrentGoalsClient.call(currentGoalSrv)) {
			for(size_t j=0;j<currentGoalSrv.response.attributes.size();j++) {
				rosplan_knowledge_msgs::KnowledgeItem attr = currentGoalSrv.response.attributes[j];
				if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT)
					goal_attributes.push_back(attr);
			}
		} else {
			ROS_ERROR("KCL: (PS) Failed to call service /kcl_rosplan/get_current_goals");
		}
		
		ROS_INFO("KCL: (PS) Update complete");
	}
} // close namespace
