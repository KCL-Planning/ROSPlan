/***************************************************************************
 *  kb_updater_navgraph.cpp - Publish information gained from navgraph
 *
 *  Created: Wed Mar 08 10:40:23 2017
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
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <diagnostic_msgs/KeyValue.h>
#include <rcll_ros_msgs/MachineInfo.h>
#include <fawkes_msgs/NavGraph.h>
#include <fawkes_msgs/NavGraphGetPairwiseCosts.h>

#include <numeric>
#include <algorithm>
#include <regex>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		n.getParam(path, var);                \
	}

class ROSPlanKbUpdaterNavGraph {
 public:
	ROSPlanKbUpdaterNavGraph(ros::NodeHandle &n)
		: n(n), static_nodes_sent_(false)
	{
		sub_navgraph_ = n.subscribe("navgraph", 10,
		                            &ROSPlanKbUpdaterNavGraph::navgraph_cb, this);
		sub_machine_info_ = n.subscribe("rcll/machine_info", 10,
		                                &ROSPlanKbUpdaterNavGraph::machine_info_cb, this);

		create_svc_update_knowledge();
		create_svc_current_knowledge();
		create_svc_current_instances();
		create_svc_navgraph_pwcosts();

		ros::NodeHandle privn("~");
		GET_CONFIG(privn, n, "distance_function", cfg_dist_func_);
		GET_CONFIG(privn, n, "machine_instance_type", cfg_machine_instance_type_);
		GET_CONFIG(privn, n, "cost_factor", cfg_cost_factor_);

		relevant_functions_ = { cfg_dist_func_ };
		get_functions();

		if (functions_.find(cfg_dist_func_) == functions_.end()) {
			ROS_ERROR("Failed to get prototype for function '%s'", cfg_dist_func_.c_str());
			throw std::runtime_error("Failed to get prototype for distance function");
		}
		if (functions_[cfg_dist_func_].typed_parameters.size() != 4) {
			ROS_ERROR("Function '%s' is not a binary function, must be of form "
			          "(name ?from-loc ?from-side ?to-loc ?to-side)",
			          cfg_dist_func_.c_str());
			throw std::runtime_error("Invalid distance function prototype, see log");			
		}

		ROS_INFO("Distance function '%s'", cfg_dist_func_.c_str());
		for (const auto &p : functions_[cfg_dist_func_].typed_parameters) {
			ROS_INFO("  Argument %s:%s", p.key.c_str(), p.value.c_str());
		}

		read_static_nodes();
		if (! static_nodes_.empty()) {
			ROS_INFO("Static nodes");
			for (const auto &n : static_nodes_) {
				ROS_INFO("  - %s -> %s", n.first.c_str(), n.second.c_str());
			}
		}
	}

	void
	create_svc_update_knowledge()
	{
		svc_update_knowledge_ =
			n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>
			("kcl_rosplan/update_knowledge_base_array", /* persistent */ true);

		ROS_INFO("Waiting for ROSPlan service update_knowledge_base");
		svc_update_knowledge_.waitForExistence();
	}

	void
	create_svc_current_knowledge()
	{
		svc_current_knowledge_ =
			n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>
			("kcl_rosplan/get_current_knowledge", /* persistent */ true);
		ROS_INFO("Waiting for ROSPlan service get_current_knowledge");
		svc_current_knowledge_.waitForExistence();
	}

	void
	create_svc_current_instances()
	{
		svc_current_instances_ =
			n.serviceClient<rosplan_knowledge_msgs::GetInstanceService>
			("kcl_rosplan/get_current_instances", /* persistent */ true);
		ROS_INFO("Waiting for ROSPlan service get_current_instances");
		svc_current_instances_.waitForExistence();
	}

	void
	create_svc_navgraph_pwcosts()
	{
		svc_navgraph_pwcosts_ =
			n.serviceClient<fawkes_msgs::NavGraphGetPairwiseCosts>
			("navgraph/get_pairwise_costs", /* persistent */ true);
		ROS_INFO("Waiting for Fawkes service navgraph/get_pairwise_costs");
		svc_navgraph_pwcosts_.waitForExistence();
	}

	void
	get_functions()
	{
		ros::service::waitForService("kcl_rosplan/get_domain_functions",ros::Duration(20));
		ros::ServiceClient func_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>
			  ("kcl_rosplan/get_domain_functions", /* persistent */ true);
		if (! func_client.waitForExistence(ros::Duration(20))) {
			ROS_ERROR("No service provider for get_domain_functions");
			return;
		}	

		rosplan_knowledge_msgs::GetDomainAttributeService func_srv;

		if (func_client.call(func_srv)) {
			for (const auto &rf : relevant_functions_) {
				const auto &i = std::find_if(func_srv.response.items.begin(), func_srv.response.items.end(),
				                             [&rf](const auto &pn) { return rf == pn.name; });
				if (i != func_srv.response.items.end()) {
					functions_[rf] = *i;
					ROS_INFO("Relevant function: %s", i->name.c_str());
				} else {
					ROS_ERROR("Failed to get function info for: %s", rf.c_str());
				}
			}
		} else {
			ROS_ERROR("Failed to call get_domain_functions");
		}
	}


	void
	read_static_nodes()
	{
		ros::NodeHandle privn("~");
		XmlRpc::XmlRpcValue value;
		privn.getParam("static_nodes", value);
		if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
			ROS_ERROR("[KB-NavGraph] Invalid configuration, 'static-nodes' not a map %i", value.getType());
			throw std::runtime_error("Invalid configuration, static nodes not a map");
		}

		for (auto &e : value) {
			if (e.second.getType() != XmlRpc::XmlRpcValue::TypeString) {
				ROS_ERROR("[KB-NavGraph] Invalid configuration, 'static-nodes/%s' no a string",
				          e.first.c_str());
				throw std::runtime_error("Invalid configuration, static node not a string");
			}

			static_nodes_[e.first] = static_cast<std::string>(e.second);
		}
	}


	void
	erase_func_values(const std::string &func_name,
	                  rosplan_knowledge_msgs::KnowledgeUpdateServiceArray &remsrv)
	{
		rosplan_knowledge_msgs::GetAttributeService srv;
		srv.request.predicate_name = func_name;
		if (! svc_current_knowledge_.isValid()) {
			create_svc_current_knowledge();
		}
		if (svc_current_knowledge_.call(srv)) {
			std::copy(srv.response.attributes.begin(), srv.response.attributes.end(),
			          std::back_inserter(remsrv.request.knowledge));
		} else {
			ROS_WARN("Failed to get old function values for '%s'", func_name.c_str());
		}
	}

	void
	machine_info_cb(const rcll_ros_msgs::MachineInfo::ConstPtr& msg)
	{
		// we can't do nothing if we don't know the graph, yet
		// that's fine, we'll get the machine info periodically anyway
		if (! last_navgraph_msg_) return;

		std::vector<std::string> nodes;
		std::transform(msg->machines.begin(), msg->machines.end(), std::back_inserter(nodes),
		               [](const rcll_ros_msgs::Machine &m) { return m.name; });
		std::sort(nodes.begin(), nodes.end());

		bool em = std::equal(nodes.begin(), nodes.end(),
		                     known_machines_.begin(), known_machines_.end());
		
		if (! em || (! static_nodes_.empty() && ! static_nodes_sent_)) {
			ROS_INFO("Machine info triggers sending of navgraph");
			std::string machine_str =
				std::accumulate(std::next(nodes.begin()), nodes.end(), nodes.front(),
				                [](std::string &s, const std::string &n) { return s + ", " + n; });
			ROS_INFO("  Received machines: %s", machine_str.c_str());

			if (! known_machines_.empty()) {
				machine_str =
					std::accumulate(std::next(known_machines_.begin()), known_machines_.end(),
					                known_machines_.front(),
					                [](std::string &s, const std::string &n) { return s + ", " + n; });
				ROS_INFO("  Known machines: %s", machine_str.c_str());
			}
			send_navgraph(nodes);
		}
	}


	void
	navgraph_cb(const fawkes_msgs::NavGraph::ConstPtr& msg)
	{
		ROS_INFO("Updating graph");
		last_navgraph_msg_ = msg;

		// we don't know any machines yet, just remember the message and don't do anything else
		if (known_machines_.empty())  return;

		// we allow to look for both mismatches before checking as this
		// will be an operation not carried out frequently.
		// Note that we assume the ordering of the nodes and edges in the incoming messages to
		// be stable. This should generally be valid knowing the code of the refbox and
		// the ROS converter.
		bool en = std::equal(msg->nodes.begin(), msg->nodes.end(),
		                     last_navgraph_msg_->nodes.begin(), last_navgraph_msg_->nodes.end(),
		                     [](const fawkes_msgs::NavGraphNode &n1, const fawkes_msgs::NavGraphNode &n2)
		                     { return n1.name == n2.name; });
		bool ee = std::equal(msg->edges.begin(), msg->edges.end(),
		                     last_navgraph_msg_->edges.begin(), last_navgraph_msg_->edges.end(),
		                     [](const fawkes_msgs::NavGraphEdge &e1, const fawkes_msgs::NavGraphEdge &e2)
		                     { return e1.from_node == e2.from_node && e1.to_node == e2.to_node; });

		if (! en || ! ee  || (! static_nodes_.empty() && ! static_nodes_sent_)) {
			ROS_INFO("Navgraph message triggers sending of navgraph");
			send_navgraph(known_machines_);
		}
	}

	/*
	// This version also splits off the team
	std::tuple<std::string, std::string, std::string>
	split_machine_name(const std::string &machine_name)
	{
		std::regex name_regex("^(C|M)-([^-]+)-(I|O)$");
		std::smatch name_match;
		if (std::regex_match(machine_name, name_match, name_regex)) {
			return std::make_tuple(name_match[1].str(), name_match[2].str(), name_match[3].str());
		}
	}
	*/

	std::tuple<std::string, std::string>
	split_machine_name(const std::string &machine_name)
	{
		std::regex name_regex("^((C|M)-[^-]+)-(I|O)$");
		std::smatch name_match;
		if (std::regex_match(machine_name, name_match, name_regex)) {
			return std::make_tuple(name_match[1].str(), name_match[3].str());
		} else {
			return std::make_tuple(machine_name, "");
		}
	}
	
	void
	send_navgraph(const std::vector<std::string> &nodes)
	{
		rosplan_knowledge_msgs::GetInstanceService srv;
		srv.request.type_name = cfg_machine_instance_type_;
		if (! svc_current_instances_.isValid()) {
			create_svc_current_instances();
		}
		if (! svc_current_instances_.call(srv)) {
			ROS_ERROR("Failed to retrieve current instances of type '%s'",
			          cfg_machine_instance_type_.c_str());
			return;
		}

		// check if there are PDDL instances for all desired machines
		std::vector<std::string> known_instances, unknown_instances;

		const std::vector<std::string> &instances = srv.response.instances;

		std::for_each(nodes.begin(), nodes.end(),
		              [&instances, &known_instances, &unknown_instances](const std::string &s) {
			              if (std::find(instances.begin(), instances.end(), s) != instances.end())
			              { known_instances.push_back(s); } else { unknown_instances.push_back(s); };
		              });

		if (! unknown_instances.empty()) {
			std::string instance_str =
				std::accumulate(std::next(unknown_instances.begin()), unknown_instances.end(),
				                unknown_instances.front(),
				                [](std::string &s, const std::string &n) { return s + ", " + n; });
			ROS_WARN("Failed to get instances for desired nodes (%s)", instance_str.c_str());
		}

		// convert instance names to input and output nodes
		std::vector<std::string> target_nodes;
		for (const auto &i : known_instances) {
			target_nodes.push_back(i + "-I");
			target_nodes.push_back(i + "-O");
		}
		{
			std::string target_str =
				std::accumulate(std::next(target_nodes.begin()), target_nodes.end(),
				                target_nodes.front(),
				                [](std::string &s, const std::string &n) { return s + ", " + n; });
			ROS_INFO("Target nodes: %s", target_str.c_str());
		}

		for (const auto &n : static_nodes_) {
			target_nodes.push_back(n.first);
		}
		
		// check if there are navgraph nodes for all desired machines
		std::vector<std::string> known_nodes, unknown_nodes;
		fawkes_msgs::NavGraph::ConstPtr msg = last_navgraph_msg_;
		
		std::for_each(target_nodes.begin(), target_nodes.end(),
		              [&msg, &known_nodes, &unknown_nodes](const std::string &s) {
			              auto i = std::find_if(msg->nodes.begin(), msg->nodes.end(),
			                                    [&s](const fawkes_msgs::NavGraphNode &n)
			                                    { return n.name == s; });
			              if (i != msg->nodes.end())
			              { known_nodes.push_back(s); } else { unknown_nodes.push_back(s); };
		              });

		if (! unknown_nodes.empty()) {
			std::string node_str =
				std::accumulate(std::next(unknown_nodes.begin()), unknown_nodes.end(),
				                unknown_nodes.front(),
				                [](std::string &s, const std::string &n) { return s + ", " + n; });
			ROS_WARN("Failed to get navgraph nodes for desired nodes (%s)", node_str.c_str());
		}

		// Call navgraph pairwise costs service
		fawkes_msgs::NavGraphGetPairwiseCosts pwc_srv;
		pwc_srv.request.nodes = known_nodes;
		if (! svc_navgraph_pwcosts_.isValid()) {
			create_svc_navgraph_pwcosts();
		}
		if (! svc_navgraph_pwcosts_.call(pwc_srv)) {
			ROS_ERROR("Failed to retrieve pairwise costs of for nodes");
			return;
		}

		if (! pwc_srv.response.ok) {
			std::string node_str;
			std::for_each(known_nodes.begin(), known_nodes.end(),
			              [&node_str](const auto &s) { node_str += s + ", "; });
			node_str.resize(node_str.size() - 2); // remove final ", "
			ROS_ERROR("Failed to get pairwise costs for nodes (%s): %s",
			          node_str.c_str(), pwc_srv.response.errmsg.c_str());
			return;
		}

		// now actually prepare the updates
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray remsrv;
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray addsrv;

		erase_func_values(cfg_dist_func_, remsrv);

		for (const fawkes_msgs::NavGraphPathCost &pc : pwc_srv.response.path_costs) {
			std::string from_node =
				(static_nodes_.find(pc.from_node) == static_nodes_.end()) ? pc.from_node : static_nodes_[pc.from_node];
			std::string to_node =
				(static_nodes_.find(pc.to_node) == static_nodes_.end()) ? pc.to_node : static_nodes_[pc.to_node];
			
			std::tuple<std::string, std::string> from = split_machine_name(from_node);
			std::tuple<std::string, std::string> to   = split_machine_name(to_node);

			std::vector<std::pair<std::string, std::string>> args =
				{ {functions_[cfg_dist_func_].typed_parameters[0].key, std::get<0>(from) },
				  {functions_[cfg_dist_func_].typed_parameters[1].key, std::get<1>(from) == "O" ? "OUTPUT" : "INPUT" },
				  {functions_[cfg_dist_func_].typed_parameters[2].key, std::get<0>(to) },
				  {functions_[cfg_dist_func_].typed_parameters[3].key, std::get<1>(to) == "O" ? "OUTPUT" : "INPUT" } };

			rosplan_knowledge_msgs::KnowledgeItem new_a;
			new_a.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
			new_a.attribute_name = cfg_dist_func_;
			for (const auto &a : args) {
				diagnostic_msgs::KeyValue kv;
				kv.key = a.first; kv.value = a.second;
				new_a.values.push_back(kv);
			}
			new_a.function_value = pc.cost * cfg_cost_factor_;
			addsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_KNOWLEDGE);
			addsrv.request.knowledge.push_back(new_a);
			ROS_INFO("Adding '(= (%s %s %s %s %s) %f)' (cost: %f * %f)", cfg_dist_func_.c_str(),
			         args[0].second.c_str(), args[1].second.c_str(),
			         args[2].second.c_str(), args[3].second.c_str(),
			         new_a.function_value, pc.cost, cfg_cost_factor_);
		}

		// execute kb updates
		if (! remsrv.request.knowledge.empty()) {
			remsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::REMOVE_KNOWLEDGE);
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(remsrv)) {
				ROS_ERROR("Failed to remove functions");
			} else {
				ROS_INFO("Removed %zu function values for '%s'", remsrv.request.knowledge.size(),
				cfg_dist_func_.c_str());
			}
		}
		if (! addsrv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(addsrv)) {
				ROS_ERROR("Failed to add functions");
				return;
			}
			ROS_INFO("Added %zu function values for '%s'", addsrv.request.knowledge.size(),
			         cfg_dist_func_.c_str());
		}

		// remember machines for which we published path lengths
		known_machines_ = nodes;
		static_nodes_sent_ = true;
	}

 private:
	ros::NodeHandle    n;

	ros::Subscriber    sub_navgraph_;
	ros::Subscriber    sub_machine_info_;
	ros::ServiceClient svc_update_knowledge_;
	ros::ServiceClient svc_current_knowledge_;
	ros::ServiceClient svc_current_instances_;
	ros::ServiceClient svc_navgraph_pwcosts_;

	std::string cfg_dist_func_;
	std::string cfg_machine_instance_type_;
	float       cfg_cost_factor_;

	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> functions_;
	std::list<std::string> relevant_functions_;

	std::vector<std::string> known_machines_;
	fawkes_msgs::NavGraph::ConstPtr last_navgraph_msg_;

	std::map<std::string, std::string> static_nodes_;
	bool static_nodes_sent_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_kb_updater_navgraph");

	ros::NodeHandle n;

	ROSPlanKbUpdaterNavGraph rosplan_kb_updater(n);
	
  ros::spin();
  
	return 0;
}
