
#include <rosplan_planning_system/ProblemGeneration/CHIMPProblemGenerator.h>

#include "rosplan_planning_system/ProblemGeneration/CHIMPProblemGenerator.h"

namespace KCL_rosplan {


	/*--------*/
	/* header */
	/*--------*/

	void CHIMPProblemGenerator::addInstances(CHIMPProblem& problem) {
		for (std::string instance : queryAllInstances()) {
			problem.addArgumentSymbol(instance);
		}
	}

	std::vector<std::string> CHIMPProblemGenerator::queryAllInstances() {
		ros::NodeHandle nh;

		// We do not need the type for CHIMPs problem format. Therefore, we can just query all instances.
		ros::ServiceClient getInstancesClient = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>(state_instance_service);
		rosplan_knowledge_msgs::GetInstanceService instanceSrv;
		if (!getInstancesClient.call(instanceSrv)) {
			ROS_ERROR("(CHIMPProblemGenerator) Failed to call service %s: %s", state_instance_service.c_str(), instanceSrv.request.type_name.c_str());
			std::vector<std::string> empty;
			return empty;
		} else {
			return instanceSrv.response.instances;
		}
	}


	std::vector<std::string> extractArgs(rosplan_knowledge_msgs::KnowledgeItem item) {
		std::vector<std::string> args;
		for (auto v : item.values) {
			args.push_back(v.value);
		}
		return args;
	}

	std::vector<CHIMPFluent> knowledgeItemsToFluents(std::vector<rosplan_knowledge_msgs::KnowledgeItem> items, CHIMPFluent::FluentType type) {
		std::vector<CHIMPFluent> fluents;
		for (auto item : items) {
			if (item.is_negative) {
				ROS_ERROR("KCL: (CHIMPProblemGenerator) CHIMP does not support negative predicates: %s", item.attribute_name.c_str());
			} else {
				CHIMPFluent fl {type, item.attribute_name, extractArgs(item)};
				fluents.push_back(fl);
			}
		}
		return fluents;
	}


	/*---------------*/
	/* initial state */
	/*---------------*/

	void CHIMPProblemGenerator::addInitialState(CHIMPProblem& problem) {

		ros::NodeHandle nh;
		ros::ServiceClient getDomainPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_predicate_service);
		ros::ServiceClient getDomainFuncsClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(domain_function_service);
		ros::ServiceClient getPropsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_proposition_service);
		ros::ServiceClient getFuncsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_function_service);
		ros::ServiceClient getTILsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_timed_knowledge_service);

		// This is required when we add time to the fluents
		// // note the time now for TILs
		// ros::Time time = ros::Time::now() + ros::Duration(1);

		// get propositions
		rosplan_knowledge_msgs::GetDomainAttributeService domainAttrSrv;
		if (!getDomainPropsClient.call(domainAttrSrv)) {
			ROS_ERROR("KCL: (CHIMPProblemGenerator) Failed to call service %s", domain_predicate_service.c_str());
			return;
		}
		
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator ait = domainAttrSrv.response.items.begin();
		for(; ait != domainAttrSrv.response.items.end(); ait++) {

			rosplan_knowledge_msgs::GetAttributeService attrSrv;
			attrSrv.request.predicate_name = ait->name;
			if (!getPropsClient.call(attrSrv)) {
				ROS_ERROR("KCL: (CHIMPProblemGenerator) Failed to call service %s: %s", state_proposition_service.c_str(), attrSrv.request.predicate_name.c_str());
			} else {
				problem.addFluents(knowledgeItemsToFluents(attrSrv.response.attributes, CHIMPFluent::FluentType::STATE));
			}

			attrSrv.request.predicate_name = ait->name;
			attrSrv.response.attributes.clear();
			if (!getTILsClient.call(attrSrv)) {
				ROS_ERROR("KCL: (CHIMPProblemGenerator) Failed to call service %s: %s", state_timed_knowledge_service.c_str(), attrSrv.request.predicate_name.c_str());
			} else {
				// TODO add initial time to fluent
				//pFile << "    (at " << (attr.initial_time - time).toSec() << " (";
				problem.addFluents(knowledgeItemsToFluents(attrSrv.response.attributes, CHIMPFluent::FluentType::STATE));
			}
		}
		
	}

	/*------*/
	/* goal */
	/*------*/

	void CHIMPProblemGenerator::addGoals(CHIMPProblem& problem) {
			
		ros::NodeHandle nh;
		ros::ServiceClient getCurrentGoalsClient = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>(state_goal_service);

		// get current goals
		rosplan_knowledge_msgs::GetAttributeService currentGoalSrv;
		if (!getCurrentGoalsClient.call(currentGoalSrv)) {
			ROS_ERROR("KCL: (CHIMPProblemGenerator) Failed to call service %s", state_goal_service.c_str());
			return;
		}

		auto fluents = knowledgeItemsToFluents(currentGoalSrv.response.attributes, CHIMPFluent::FluentType::TASK);
		for (auto fl : fluents) {
			problem.addTask(fl);
		}
	}

	void CHIMPProblemGenerator::makeProblem(std::ofstream &pFile) {
		CHIMPProblem problem;
		addInstances(problem);
		addInitialState(problem);
		addGoals(problem);
		problem.generateProblem(pFile);
	};

} // close namespace
