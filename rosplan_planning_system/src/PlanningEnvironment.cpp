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

	/*----------------------*/
	/* updating environment */
	/*----------------------*/

	void PlanningEnvironment::clear() {

		// clear old problem
		type_object_map.clear();
		object_type_map.clear();
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
		for(size_t t=0; t<domain_parser.domain_types.size(); t++) {

			rosplan_knowledge_msgs::GetInstanceService instanceSrv;
			instanceSrv.request.type_name = domain_parser.domain_types[t];
			type_object_map[domain_parser.domain_types[t]];

			if (GetInstancesClient.call(instanceSrv)) {
				for(size_t i=0;i<instanceSrv.response.instances.size();i++) {

					// add new instance (popf converts names to lowercase)
					std::string name = instanceSrv.response.instances[i];
					name_map[KCL_rosplan::toLowerCase(name)] = name;
					type_object_map[domain_parser.domain_types[t]].push_back(name);
					object_type_map[name] = domain_parser.domain_types[t];
				}
			} else {
				ROS_ERROR("KCL: (PS) Failed to call service /kcl_rosplan/get_instances: %s", instanceSrv.request.type_name.c_str());
			}
		}


		// get domain attributes and functions
		std::map< std::string, PDDLAtomicFormula>::iterator ait;
		for(ait = domain_parser.domain_predicates.begin(); ait != domain_parser.domain_predicates.end(); ait++) {
			rosplan_knowledge_msgs::GetAttributeService domainAttrSrv;
			domainAttrSrv.request.predicate_name = ait->first;
			if (GetDomainAttrsClient.call(domainAttrSrv)) {
				for(size_t j=0;j<domainAttrSrv.response.attributes.size();j++) {
					rosplan_knowledge_msgs::KnowledgeItem attr = domainAttrSrv.response.attributes[j];
					if(attr.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT && attr.attribute_name.compare(ait->first)==0)
						domain_attributes.push_back(attr);
				}
			} else {
				ROS_ERROR("KCL: (PS) Failed to call service /kcl_rosplan/get_domain_attributes %s", domainAttrSrv.request.predicate_name.c_str());
			}
		}
		for(ait = domain_parser.domain_functions.begin(); ait != domain_parser.domain_functions.end(); ait++) {
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
	}
} // close namespace
