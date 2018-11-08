
#include <rosplan_planning_system/ProblemGeneration/PDDLProblemGenerator.h>

#include "rosplan_planning_system/ProblemGeneration/PDDLProblemGenerator.h"

namespace KCL_rosplan {


	/*--------*/
	/* header */
	/*--------*/

	void PDDLProblemGenerator::makeHeader(std::ofstream &pFile) {

		// setup service calls
		ros::NodeHandle nh;


		ros::ServiceClient getNameClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainNameService>(domain_name_service);
		ros::ServiceClient getTypesClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainTypeService>(domain_type_service);
		ros::ServiceClient getInstancesClient = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(state_instance_service);

		// get domain name
		rosplan_knowledge_msgs::GetDomainNameService nameSrv;
		if (!getNameClient.call(nameSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", domain_name_service.c_str());
		}

		pFile << "(define (problem task)" << std::endl;
		pFile << "(:domain " << nameSrv.response.domain_name << ")" << std::endl;

		/* objects */
		pFile << "(:objects" << std::endl;

		// get types
		rosplan_knowledge_msgs::GetDomainTypeService typeSrv;
		if (!getTypesClient.call(typeSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", domain_type_service.c_str());
		}

		// get instances of each type
		for(size_t t=0; t<typeSrv.response.types.size(); t++) {

			rosplan_knowledge_msgs::GetInstanceService instanceSrv;
			instanceSrv.request.type_name = typeSrv.response.types[t];

			if (!getInstancesClient.call(instanceSrv)) {
				ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s", state_instance_service.c_str(), instanceSrv.request.type_name.c_str());
			} else {
				if(instanceSrv.response.instances.size() == 0) continue;
				pFile << "    ";
				for(size_t i=0;i<instanceSrv.response.instances.size();i++) {
					pFile << instanceSrv.response.instances[i] << " ";
				}
				pFile << "- " << typeSrv.response.types[t] << std::endl;
			}
		}

		pFile << ")" << std::endl;
	}

	/*---------------*/
	/* initial state */
	/*---------------*/

	void PDDLProblemGenerator::makeInitialState(std::ofstream &pFile) {

		ros::NodeHandle nh;
		ros::ServiceClient getDomainPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_predicate_service);
		ros::ServiceClient getDomainFuncsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_function_service);
		ros::ServiceClient getPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_proposition_service);
		ros::ServiceClient getFuncsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_function_service);
		ros::ServiceClient getTILsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_timed_knowledge_service);

		// note the time now for TILs
		ros::Time time = ros::Time::now() + ros::Duration(1);

		pFile << "(:init" << std::endl;

		// get propositions
		rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;
		if (!getDomainPropsClient.call(domainAttrSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", domain_predicate_service.c_str());
		} else {

			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait = domainAttrSrv.response.items.begin();
			for(; ait != domainAttrSrv.response.items.end(); ait++) {

				rosplan_knowledge_msgs::GetAttributeService attrSrv;
				attrSrv.request.predicate_name = ait->name;
				if (!getPropsClient.call(attrSrv)) {
					ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s", state_proposition_service.c_str(), attrSrv.request.predicate_name.c_str());
				} else {

					for(size_t i=0;i<attrSrv.response.attributes.size();i++) {

						rosplan_knowledge_msgs::KnowledgeItem attr = attrSrv.response.attributes[i];

						pFile << "    (";		
						
						//Check if the attribute is negated
						if(attr.is_negative) pFile << "not (";

						pFile << attr.attribute_name;
						for(size_t j=0; j<attr.values.size(); j++) {
							pFile << " " << attr.values[j].value;
						}
						pFile << ")";
						
						if(attr.is_negative) pFile << ")";

						pFile << std::endl;
					}
				}
				pFile << std::endl;

				attrSrv.request.predicate_name = ait->name;
				attrSrv.response.attributes.clear();
				if (!getTILsClient.call(attrSrv)) {
					ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s", state_timed_knowledge_service.c_str(), attrSrv.request.predicate_name.c_str());
				} else {
					if(attrSrv.response.attributes.size() == 0) continue;

					for(size_t i=0;i<attrSrv.response.attributes.size();i++) {

						rosplan_knowledge_msgs::KnowledgeItem attr = attrSrv.response.attributes[i];

						pFile << "    (at " << (attr.initial_time - time).toSec() << " (";
						
						//Check if the attribute is negated
						if(attr.is_negative) pFile << "not (";

						pFile << attr.attribute_name;
						for(size_t j=0; j<attr.values.size(); j++) {
							pFile << " " << attr.values[j].value;
						}
						pFile << "))";
						
						if(attr.is_negative) pFile << ")";

						pFile << std::endl;
					}
				}
				pFile << std::endl;
			}
		}

		// get functions
		if (!getDomainFuncsClient.call(domainAttrSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", domain_function_service.c_str());
		} else {

			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait = domainAttrSrv.response.items.begin();
			for(; ait != domainAttrSrv.response.items.end(); ait++) {

				rosplan_knowledge_msgs::GetAttributeService attrSrv;
				attrSrv.request.predicate_name = ait->name;
				if (!getFuncsClient.call(attrSrv)) {
					ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s: %s", state_function_service.c_str(), attrSrv.request.predicate_name.c_str());
				} else {

					for(size_t i=0;i<attrSrv.response.attributes.size();i++) {

						rosplan_knowledge_msgs::KnowledgeItem attr = attrSrv.response.attributes[i];

						pFile << "    (";

						if(time < attr.initial_time) {
							pFile << "at " << (attr.initial_time - time).toSec() << " (";
						}

						pFile << "= (";

						pFile << attr.attribute_name;
						for(size_t j=0; j<attr.values.size(); j++) {
							pFile << " " << attr.values[j].value;
						}

						pFile << ") " << attr.function_value << ")";

						if(time < attr.initial_time) {
							pFile << ")";
						}

						pFile << std::endl;
					}
				}
				pFile << std::endl;
			}
		}

		pFile << ")" << std::endl;
	}

	/*------*/
	/* goal */
	/*------*/

	void PDDLProblemGenerator::makeGoals(std::ofstream &pFile) {
			
		ros::NodeHandle nh;
		ros::ServiceClient getCurrentGoalsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_goal_service);

		pFile << "(:goal (and" << std::endl;

		// get current goals
		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		if (!getCurrentGoalsClient.call(currentGoalSrv)) {
			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", state_goal_service.c_str());
		} else {

			for(size_t i=0;i<currentGoalSrv.response.attributes.size();i++) {
				rosplan_knowledge_msgs::KnowledgeItem attr = currentGoalSrv.response.attributes[i];
				if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

                    if(attr.is_negative){pFile << "    (not("+ attr.attribute_name;}
                    else {pFile << "    (" + attr.attribute_name;}

                    for(size_t j=0; j<attr.values.size(); j++) {
                        pFile << " " << attr.values[j].value;
                    }

                    if(attr.is_negative){pFile << ")";}

                    pFile << ")" << std::endl;


				}
				if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INEQUALITY) {

                    // do we need comparison negation? remove if not
                    if(attr.is_negative)pFile << "    (not(";
                        else pFile << "    (";

                    switch(attr.ineq.comparison_type){
                        case 0: pFile << "> "; break;
                        case 1: pFile << ">= "; break;
                        case 2: pFile << "<" ; break;
                        case 3: pFile << "<= "; break;
                        case 4: pFile << "=" ; break;
                    }

					printExpression(pFile, attr.ineq.LHS);
					printExpression(pFile, attr.ineq.RHS);

					if(attr.is_negative){pFile << ")";}
					pFile << ")" << std::endl;
				}
			}
		}
		pFile << "))" << std::endl;
	}


	/*--------*/
	/* metric */
	/*--------*/

	void PDDLProblemGenerator::makeMetric(std::ofstream &pFile) {

		ros::NodeHandle nh;

		// get current metric
		ros::ServiceClient getCurrentMetricClient = nh.serviceClient<rosplan_knowledge_msgs::GetMetricService>(state_metric_service);
		rosplan_knowledge_msgs::GetMetricService currentMetricSrv;
		if (!getCurrentMetricClient.call(currentMetricSrv)) {

			ROS_ERROR("KCL: (PDDLProblemGenerator) Failed to call service %s", state_metric_service.c_str());

		} else {

			rosplan_knowledge_msgs::KnowledgeItem metric = currentMetricSrv.response.metric;
            if (metric.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::EXPRESSION) {
					
				pFile << "(:metric " << metric.optimization;
				std::vector<int> operand_count;
				printExpression(pFile, metric.expr);

				pFile << ")" << std::endl;
            }
        }
	}

	void PDDLProblemGenerator::printExpression(std::ofstream &pFile, rosplan_knowledge_msgs::ExprComposite & e) {

		std::vector<rosplan_knowledge_msgs::ExprBase> tokens = e.tokens;
		bool second_operand = false;
		int depth = 0;
		for(int i=0; i<tokens.size(); i++) {
			rosplan_knowledge_msgs::ExprBase token = tokens[i];

			pFile << " ";

			switch(token.expr_type) {
			case rosplan_knowledge_msgs::ExprBase::OPERATOR:

				switch(token.op) {
					case rosplan_knowledge_msgs::ExprBase::ADD: pFile << "(+"; break;
					case rosplan_knowledge_msgs::ExprBase::SUB: pFile << "(-"; break;
					case rosplan_knowledge_msgs::ExprBase::MUL: pFile << "(*"; break;
					case rosplan_knowledge_msgs::ExprBase::DIV: pFile << "(/"; break;
				}
				second_operand = false;
				depth++;
				break;

			case rosplan_knowledge_msgs::ExprBase::CONSTANT:

				pFile << token.constant;
				break;

			case rosplan_knowledge_msgs::ExprBase::FUNCTION:

				pFile << "(" << token.function.name;
				for(size_t j=0; j<token.function.typed_parameters.size(); j++) {
					pFile << " " << token.function.typed_parameters[j].value;
				}
				pFile << ")";
				break;

			case rosplan_knowledge_msgs::ExprBase::SPECIAL:

				switch(token.special_type) {
					case rosplan_knowledge_msgs::ExprBase::HASHT:		pFile << "#t";			break;
					case rosplan_knowledge_msgs::ExprBase::TOTAL_TIME:	pFile << "(total-time)";	break;
					case rosplan_knowledge_msgs::ExprBase::DURATION:	pFile << "?duration";	break;
				}
				break;
			}

			if(second_operand && token.expr_type!=rosplan_knowledge_msgs::ExprBase::OPERATOR) {
				second_operand = true;
				pFile << ")";
				depth--;
			}

			if(token.expr_type!=rosplan_knowledge_msgs::ExprBase::OPERATOR) {
				second_operand = true;
			}
		}

		while(depth>0) {
			pFile << ")";
			depth--;
		}
	}

	void PDDLProblemGenerator::makeProblem(std::ofstream &pFile) {
		makeHeader(pFile);
		makeInitialState(pFile);
		makeGoals(pFile);
		makeMetric(pFile);

		// add end of problem file
		pFile << ")" << std::endl;
	};

} // close namespace
