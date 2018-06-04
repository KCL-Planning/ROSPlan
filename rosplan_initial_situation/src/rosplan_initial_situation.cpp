/***************************************************************************
 *  rosplan_initial_situation.cpp - Create initial situation from config
 *
 *  Created: Fri Feb 24 17:56:41 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <ros/ros.h>

#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorService.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <map>
#include <string>
#include <vector>
#include <list>

#include <mutex>
#include <condition_variable>

class ROSPlanInitialSituation {
	typedef std::vector<std::string> PredicateArgumentList;
	typedef std::list<PredicateArgumentList> PredicateInstanceList;
	typedef std::map<std::string, PredicateInstanceList> PredicateMap;

	typedef std::list<std::string> ObjectInstanceList;
	typedef std::map<std::string, ObjectInstanceList> ObjectMap;
	
public:
	ROSPlanInitialSituation(ros::NodeHandle &n)
		: n(n)
	{
		ros::NodeHandle privn("~");

		if (! privn.getParam("clear_kb", cfg_clear_kb_)) {
			n.getParam("clear_kb", cfg_clear_kb_);
		}
		if (! privn.getParam("start_planning", cfg_start_planning_)) {
			n.getParam("start_planning", cfg_start_planning_);
		}

		if (privn.hasParam("objects")) {
			read_config_objects(privn, "objects", objects_);
		} else {
			ROS_INFO("[RP-IniSit] No objects have been specified");
		}

		if (privn.hasParam("init")) {
			read_config_predicates(privn, "init", predicates_init_);
		} else {
			ROS_INFO("[RP-IniSit] No initial attributes have been specified");
		}

		if (privn.hasParam("goal")) {
			read_config_predicates(privn, "goal", predicates_goal_);
		} else {
			ROS_INFO("[RP-IniSit] No goal has been specified");
		}

		pub_planning_cmd_ =
			n.advertise<std_msgs::String>("kcl_rosplan/planning_commands", 10, true);

		ROS_INFO("[RP-IniSit] Waiting for MongoDB (actually ROSPlan KB)");
		ros::service::waitForService("/message_store/delete", -1);

		ROS_INFO("[RP-IniSit] Waiting for planning system to become ready");
		sub_ps_state_ = n.subscribe("kcl_rosplan/system_state", 10,
		                            &ROSPlanInitialSituation::ps_state_cb, this);
	}

	void
	create_svc_update_knowledge()
	{
		svc_update_knowledge_ =
			n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>
			("kcl_rosplan/update_knowledge_base_array", /* persistent */ true);

		ROS_INFO("[RP-IniSit] Waiting for ROSPlan service update_knowledge_base");
		svc_update_knowledge_.waitForExistence();
	}

	void
	ps_state_cb(const std_msgs::String::ConstPtr& msg)
	{
		if (msg->data == "Ready") {
			ROS_INFO("[RB-IniSit] Planning System ready, proceeding");

			get_predicates();

			if (! verify_predicates(predicates_init_) ||
			    ! verify_predicates(predicates_goal_) )
			{
				ROS_ERROR("[RP-IniSit] ***** CRITICAL ERROR: failed to verify predicates");
				ROS_ERROR("[RP-IniSit] ***** CRITICAL ERROR: shutting down without adding to KB");
				ros::shutdown();
				return;
			}

			create_svc_update_knowledge();

			if (cfg_clear_kb_) kb_clear_all();

			kb_add_objects(objects_);
			kb_add_predicates(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_KNOWLEDGE,
		                  predicates_init_);
			kb_add_predicates(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_GOAL,
			                  predicates_goal_);

			if (cfg_start_planning_) ps_plan_async();

			ROS_INFO("[RP-IniSit] *** Job done, quitting ***");
			ros::shutdown();

		} else {
			ROS_INFO("[RP-IniSit] Planning System reports state '%s', will keep waiting", msg->data.c_str());
		}
	}

	void
	read_config_objects(ros::NodeHandle &n, std::string key, ObjectMap &elements)
	{
		XmlRpc::XmlRpcValue value;
		n.getParam(key, value);
		if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
			ROS_ERROR("[RP-IniSit] Invalid configuration, %s not a map", key.c_str());
			throw std::runtime_error("Invalid configuration, objects not a map");
		}

		ROS_INFO("[RP-IniSit] Objects");
		for (auto &e : value) {
			ObjectInstanceList instance_list;
			if (e.second.getType() == XmlRpc::XmlRpcValue::TypeString) {
				ROS_INFO("[RP-IniSit]   %s - %s", static_cast<std::string>(e.second).c_str(), e.first.c_str());
				instance_list.push_back(static_cast<std::string>(e.second));
			} else if (e.second.getType() == XmlRpc::XmlRpcValue::TypeArray) {
				for (int i = 0; i < e.second.size(); ++i) {
					ROS_INFO("[RP-IniSit]   %s - %s", static_cast<std::string>(e.second[i]).c_str(), e.first.c_str());
					instance_list.push_back(static_cast<std::string>(e.second[i]));
				}
			}
			elements[e.first] = instance_list;
		}
	}

	void
	read_config_predicates(ros::NodeHandle &n, std::string key, PredicateMap &elements)
	{
		XmlRpc::XmlRpcValue value;
		n.getParam(key, value);
		if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
			ROS_ERROR("[RP-IniSit] Invalid configuration, %s not a map", key.c_str());
			throw std::runtime_error("Invalid configuration");
		}
		ROS_INFO("[RP-IniSit] Reading %s predicates", key.c_str());
		for (auto &e : value) {
			if (e.second.getType() != XmlRpc::XmlRpcValue::TypeArray) {
				ROS_ERROR("[RP-IniSit] Invalid configuration, %s predicate %s not an array", key.c_str(), e.first.c_str());
				throw std::runtime_error("Invalid configuration, invalid predicate, see log");
			}
			PredicateInstanceList predicate_list;
			for (int i = 0; i < e.second.size(); ++i) {
				PredicateArgumentList predarg_list;
				XmlRpc::XmlRpcValue &f = e.second[i];
				if (f.getType() == XmlRpc::XmlRpcValue::TypeString) {
					ROS_INFO("[RP-IniSit]   (%s %s)",
					         e.first.c_str(), static_cast<std::string>(f).c_str());
					predarg_list.push_back(static_cast<std::string>(f));
				} else if (e.second.getType() == XmlRpc::XmlRpcValue::TypeArray) {
					std::string s;
					for (int j = 0; j < f.size(); ++j) {
						s += " " + static_cast<std::string>(f[j]);
						predarg_list.push_back(static_cast<std::string>(f[j]));
					}
					ROS_INFO("[RP-IniSit]   (%s%s)", e.first.c_str(), s.c_str());
				}
				predicate_list.push_back(predarg_list);
			}
			elements[e.first] = predicate_list;
		}
	}
	
	void collect_predicates(std::set<std::string> &predicate_names,
	                        const PredicateMap &pm)
	{
		std::for_each(pm.begin(), pm.end(),
		              [&predicate_names](const auto &f) { predicate_names.insert(f.first); });
	}

	void
	get_predicates()
	{
		std::set<std::string> predicate_names;
		collect_predicates(predicate_names, predicates_init_);
		collect_predicates(predicate_names, predicates_goal_);

		ros::ServiceClient pred_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>
			  ("kcl_rosplan/get_domain_predicate_details", /* persistent */ true);
		if (! pred_client.waitForExistence(ros::Duration(20))) {
			ROS_ERROR("[RP-IniSit] No service provider for get_domain_predicate_details");
			return;
		}

		for (const auto &pn : predicate_names) {
			rosplan_knowledge_msgs::GetDomainPredicateDetailsService pred_srv;
			pred_srv.request.name = pn;
			if (pred_client.call(pred_srv)) {
				std::string pred_str;
				std::for_each(pred_srv.response.predicate.typed_parameters.begin(),
				              pred_srv.response.predicate.typed_parameters.end(),
				              [&pred_str](const auto &kv) { pred_str += " " + kv.key + ":" + kv.value; });
				ROS_INFO("[RP-IniSit] Relevant predicate: (%s%s)", pn.c_str(), pred_str.c_str());
				predicates_[pn] = pred_srv.response.predicate;
			}
		}
	}

	bool
	verify_predicates(const PredicateMap &pm)
	{
		bool rv = true;
		for (const auto &p : pm) {
			if (predicates_.find(p.first) == predicates_.end()) {
				ROS_WARN("[RP-IniSit] Could not get information for predicate '%s'", p.first.c_str());
				rv = false;
			}
		}
		return rv;
	}

	void
	kb_clear_all()
	{
		if (pub_planning_cmd_.getNumSubscribers() > 0) {
			std_msgs::String msg;
			msg.data = "cancel";
			pub_planning_cmd_.publish(msg);
		} else {
			ROS_WARN("[RP-IniSit] No subscriber for planning command, cannot cancel");
		}

		ros::ServiceClient clear_client =
			n.serviceClient<std_srvs::Empty>("kcl_rosplan/clear_knowledge_base");
		if (! clear_client.waitForExistence(ros::Duration(20))) {
			ROS_WARN("[RP-IniSit] No service provider for clear_knowledge_base, cannot clear");
			return;
		}
		std_srvs::Empty srv;
		clear_client.call(srv);
	}

	void
	ps_plan_async()
	{
		if (pub_planning_cmd_.getNumSubscribers() >= 0) {
			std_msgs::String msg;
			msg.data = "plan";
			pub_planning_cmd_.publish(msg);
		} else {
			ROS_WARN("[RP-IniSit] No subscriber for planning command, cannot plan");
		}
	}

	void
	kb_add_objects(ObjectMap &objects)
	{
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray srv;
		for (const auto &o : objects) {
			for (const auto &i : o.second) {
				rosplan_knowledge_msgs::KnowledgeItem new_i;
				new_i.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				new_i.instance_type = o.first;
				new_i.instance_name = i;
				srv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_KNOWLEDGE);
				srv.request.knowledge.push_back(new_i);
			}
		}
		if (! srv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(srv)) {
				ROS_ERROR("[RP-IniSit] Failed to add objects");
				return;
			}
		}
	}

	void
	kb_add_predicates(int update_type, PredicateMap &pmap)
	{
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray srv;
		for (const auto &p : pmap) {
			// first: predicate type name, second: PredicateInstanceList
			for (const PredicateArgumentList &i : p.second) {
				// each i: PredicateArgumentList
				if (i.size() != predicates_[p.first].typed_parameters.size()) {
					std::string param_str;
					std::for_each(i.begin(), i.end(),
					              [&param_str](const auto &s) { param_str += " '" + s + "'"; });
					ROS_ERROR("[RP-IniSit] Cannot add predicate instance (%s%s), "
					          "got %zu parameters, expected %zu",
					          p.first.c_str(), param_str.c_str(),
					          i.size(), predicates_[p.first].typed_parameters.size());
					continue;
				}
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item.attribute_name = p.first;
				for (int j = 0; j < i.size(); ++j) {
					diagnostic_msgs::KeyValue pair;
					pair.key = predicates_[p.first].typed_parameters[j].key;
					pair.value = i[j];
					item.values.push_back(pair);
				}
				srv.request.update_type.push_back(update_type);
				srv.request.knowledge.push_back(item);
			}
		}
		if (! srv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(srv)) {
				ROS_ERROR("Failed to add predicates");
				return;
			}
		}
	}

 private:
	ros::NodeHandle    n;

	bool cfg_clear_kb_;
	bool cfg_start_planning_;
	
	ros::Publisher     pub_planning_cmd_;
	ros::Subscriber    sub_ps_state_;
	ros::ServiceClient svc_update_knowledge_;
	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates_;

	PredicateMap predicates_init_;
	PredicateMap predicates_goal_;
	ObjectMap    objects_;

	std::mutex mtx_wait_ready_;
	std::condition_variable cdv_wait_ready_;
	std::string ps_state_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_initial_situation");
	ros::NodeHandle n;
	ROSPlanInitialSituation rosplan_inisit(n);
  ros::spin();
  
	return 0;
}
