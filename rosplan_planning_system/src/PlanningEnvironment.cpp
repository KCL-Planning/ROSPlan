#include "PlanningEnvironment.h"
#include "rosplan_knowledge_msgs/InstanceService.h"
#include "rosplan_knowledge_msgs/AttributeService.h"
#include "VALfiles/ptree.h"
#include "FlexLexer.h"
#include <fstream>
#include <sstream>
#include <string>
#include <cstdio>
#include <iostream>

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

	void PlanningEnvironment::clearDomain() {

		domainTypes.clear();
		domainPredicates.clear();
		domainOperators.clear();
		domainOperatorTypes.clear();
		domainFunctions.clear();
	}

	/**
	 * read domain: used for writing the PDDL problem file and then for constructing the dispatch
	 * messages.
	 */
	void PlanningEnvironment::parseDomain(const std::string domainPath) {
		
		// clear old domain
		clearDomain();

		std::string domainFileName = (domainPath);
		ROS_INFO("KCL: Parsing domain: %s.", domainFileName.c_str());

		// save filename for VAL
		std::vector<char> writable(domainFileName.begin(), domainFileName.end());
		writable.push_back('\0');
		current_filename = &writable[0];

		// parse domain
		VAL::current_analysis = &VAL::an_analysis;
		std::ifstream domainFile;
		domainFile.open(domainFileName.c_str());
		yydebug = 0;

		VAL::yfl = new yyFlexLexer;

		if (domainFile.bad()) {
			ROS_ERROR("KCL: Failed to open domain file.");
			line_no = 0;
			VAL::log_error(VAL::E_FATAL,"Failed to open file");
		} else {
			line_no = 1;
			VAL::yfl->switch_streams(&domainFile,&std::cout);
			yyparse();

			// domain name
			VAL::domain* domain = VAL::current_analysis->the_domain;
			domainName = domain->name;

			// types
			VAL::pddl_type_list* types = domain->types;
			for (VAL::pddl_type_list::const_iterator ci = types->begin(); ci != types->end(); ci++) {
				const VAL::pddl_type* type = *ci;
				domainTypes.push_back(type->getName());
			}

			// predicates
			VAL::pred_decl_list* predicates = domain->predicates;
			if(predicates) {
				for (VAL::pred_decl_list::const_iterator ci = predicates->begin(); ci != predicates->end(); ci++) {
					const VAL::pred_decl* predicate = *ci;
					// predicate name
					domainPredicates[predicate->getPred()->symbol::getName()];
					// parameters
					for (VAL::var_symbol_list::const_iterator vi = predicate->getArgs()->begin(); vi != predicate->getArgs()->end(); vi++) {
						const VAL::var_symbol* var = *vi;
						domainPredicates[predicate->getPred()->symbol::getName()].push_back(var->pddl_typed_symbol::getName());
					}
				}
			}

			// functions
			VAL::func_decl_list* functions = domain->functions;
			if(functions) {
				for (VAL::func_decl_list::const_iterator ci = functions->begin(); ci != functions->end(); ci++) {
					const VAL::func_decl* function = *ci;
					// function name
					domainFunctions[function->getFunction()->symbol::getName()];
					// parameters
					for (VAL::var_symbol_list::const_iterator vi = function->getArgs()->begin(); vi != function->getArgs()->end(); vi++) {
						const VAL::var_symbol* var = *vi;
						domainFunctions[function->getFunction()->symbol::getName()].push_back(var->pddl_typed_symbol::getName());
					}
				}
			}

			// operators
			VAL::operator_list* operators = domain->ops;
			for (VAL::operator_list::const_iterator ci = operators->begin(); ci != operators->end(); ci++) {			
				const VAL::operator_* op = *ci;
				// operator name
				domainOperators[op->name->symbol::getName()];
				// parameters
				for (VAL::var_symbol_list::const_iterator vi = op->parameters->begin(); vi != op->parameters->end(); vi++) {
					const VAL::var_symbol* var = *vi;
					domainOperators[op->name->symbol::getName()].push_back(var->pddl_typed_symbol::getName());
					domainOperatorTypes[op->name->symbol::getName()].push_back(var->pddl_typed_symbol::type->getName());
				}
				// preconditions
				const VAL::goal* precondition = op->precondition;
				domainOperatorPreconditionMap[op->name->symbol::getName()];
				parsePrecondition(op->name->symbol::getName(), precondition, false);
			}
		}
		domainFile.close();
		delete VAL::yfl;
	}

	/**
	 * parse a precondition recursively.
	 * The final simple preconditions are stored in domainOperatorPreconditionMap.
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
			domainOperatorPreconditionMap[opName].push_back(precondition);
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
		typeObjectMap.clear();
		objectTypeMap.clear();
		instanceAttributes.clear();
		domainAttributes.clear();
		goalAttributes.clear();
	}

	/**
	 * requests all the information required to build a problem instance.
	 */
	void PlanningEnvironment::update(ros::NodeHandle nh) {

		// clear old problem
		clear();

		// setup service calls
		ros::ServiceClient GetInstancesClient = nh.serviceClient<rosplan_knowledge_msgs::InstanceService>("/kcl_rosplan/get_instances");
		ros::ServiceClient GetInstanceAttrsClient = nh.serviceClient<rosplan_knowledge_msgs::AttributeService>("/kcl_rosplan/get_instance_attributes");
		ros::ServiceClient GetDomainAttrsClient = nh.serviceClient<rosplan_knowledge_msgs::AttributeService>("/kcl_rosplan/get_domain_attributes");
		ros::ServiceClient GetDomainFuncsClient = nh.serviceClient<rosplan_knowledge_msgs::AttributeService>("/kcl_rosplan/get_domain_functions");
		ros::ServiceClient GetCurrentGoalsClient = nh.serviceClient<rosplan_knowledge_msgs::AttributeService>("/kcl_rosplan/get_current_goals");

		// for each type fetch instances
		for(size_t t=0; t<domainTypes.size(); t++) {

			rosplan_knowledge_msgs::InstanceService instanceSrv;
			instanceSrv.request.type_name = domainTypes[t];
			typeObjectMap[domainTypes[t]];

			if (GetInstancesClient.call(instanceSrv)) {
				for(size_t i=0;i<instanceSrv.response.instances.size();i++) {

					// add new instance (popf converts names to lowercase)
					std::string name = instanceSrv.response.instances[i];
					name_map[KCL_rosplan::toLowerCase(name)] = name;
					typeObjectMap[domainTypes[t]].push_back(name);
					objectTypeMap[name] = domainTypes[t];

					// get instance attributes
					rosplan_knowledge_msgs::AttributeService instanceAttrSrv;
					instanceAttrSrv.request.instance_name = name;
					instanceAttrSrv.request.type_name = domainTypes[t];
					if (GetInstanceAttrsClient.call(instanceAttrSrv)) {
						for(size_t j=0;j<instanceAttrSrv.response.attributes.size();j++) {
							// if knowledge item corresponds to an attribute of this object, then store it.
							rosplan_knowledge_msgs::KnowledgeItem attr = instanceAttrSrv.response.attributes[j];
							if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE_ATTRIBUTE
									&& attr.instance_type.compare(domainTypes[t])==0
									&& attr.instance_name.compare(name)==0)
								instanceAttributes.push_back(attr);
						}
					} else {
						ROS_ERROR("KCL: Failed to call service /kcl_rosplan/get_instance_attributes: %s %s",
							instanceAttrSrv.request.type_name.c_str(), instanceAttrSrv.request.instance_name.c_str());
					}
				}
			} else {
				ROS_ERROR("KCL: Failed to call service /kcl_rosplan/get_instances: %s", instanceSrv.request.type_name.c_str());
			}
		}


		// get domain attributes and functions
		std::map<std::string,std::vector<std::string> >::iterator ait;
		for(ait = domainPredicates.begin(); ait != domainPredicates.end(); ait++) {
			rosplan_knowledge_msgs::AttributeService domainAttrSrv;
			domainAttrSrv.request.predicate_name = ait->first;
			if (GetDomainAttrsClient.call(domainAttrSrv)) {
				for(size_t j=0;j<domainAttrSrv.response.attributes.size();j++) {
					rosplan_knowledge_msgs::KnowledgeItem attr = domainAttrSrv.response.attributes[j];
					if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE && attr.attribute_name.compare(ait->first)==0)
						domainAttributes.push_back(attr);
				}
			} else {
				ROS_ERROR("KCL: Failed to call service /kcl_rosplan/get_domain_attributes %s", domainAttrSrv.request.predicate_name.c_str());
			}
		}
		for(ait = domainFunctions.begin(); ait != domainFunctions.end(); ait++) {
			rosplan_knowledge_msgs::AttributeService domainAttrSrv;
			domainAttrSrv.request.predicate_name = ait->first;
			if (GetDomainAttrsClient.call(domainAttrSrv)) {
				for(size_t j=0;j<domainAttrSrv.response.attributes.size();j++) {
					rosplan_knowledge_msgs::KnowledgeItem attr = domainAttrSrv.response.attributes[j];
					if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_FUNCTION && attr.attribute_name.compare(ait->first)==0)
						domainAttributes.push_back(attr);
				}
			} else {
				ROS_ERROR("KCL: Failed to call service /kcl_rosplan/get_domain_attributes %s", domainAttrSrv.request.predicate_name.c_str());
			}
		}

		// get current goals
		rosplan_knowledge_msgs::AttributeService currentGoalSrv;
		if (GetCurrentGoalsClient.call(currentGoalSrv)) {
			for(size_t j=0;j<currentGoalSrv.response.attributes.size();j++) {
				rosplan_knowledge_msgs::KnowledgeItem attr = currentGoalSrv.response.attributes[j];
				if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::DOMAIN_ATTRIBUTE)
					goalAttributes.push_back(attr);
			}
		} else {
			ROS_ERROR("KCL: Failed to call service /kcl_rosplan/get_current_goals");
		}
	}
} // close namespace
