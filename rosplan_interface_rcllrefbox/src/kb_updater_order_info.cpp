/***************************************************************************
 *  rosplan_interface_rcllrefbox.cpp - Referee box actions
 *
 *  Created: Wed Feb 16 22:37:18 2017
 *  Copyright  2017  Tim Niemueller [www.niemueller.de] and Erez Karpas
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
#include <rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <rosplan_knowledge_msgs/GetDomainAttributeService.h>
#include <diagnostic_msgs/KeyValue.h>
#include <rcll_ros_msgs/Order.h>
#include <rcll_ros_msgs/OrderInfo.h>
#include <rcll_ros_msgs/ProductColor.h>

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		n.getParam(path, var);                \
	}

class ROSPlanKbUpdaterOrderInfo {
 public:
	ROSPlanKbUpdaterOrderInfo(ros::NodeHandle &n)
		: n(n)
	{
		sub_order_info_ = n.subscribe("rcll/order_info", 10,
		                                &ROSPlanKbUpdaterOrderInfo::order_info_cb, this);

		create_svc_update_knowledge();
		create_svc_current_knowledge();
		create_svc_current_instances();

		ros::NodeHandle privn("~");
		GET_CONFIG(privn, n, "order_complexity_predicate", order_complexity_predicate_);
		GET_CONFIG(privn, n, "order_base_color_predicate", order_base_color_predicate_);
		GET_CONFIG(privn, n, "order_ring1_color_predicate", order_ring1_color_predicate_);
		GET_CONFIG(privn, n, "order_ring2_color_predicate", order_ring2_color_predicate_);
		GET_CONFIG(privn, n, "order_ring3_color_predicate", order_ring3_color_predicate_);
		GET_CONFIG(privn, n, "order_cap_color_predicate", order_cap_color_predicate_);
		GET_CONFIG(privn, n, "order_gate_predicate", order_gate_predicate_);
		GET_CONFIG(privn, n, "order_delivery_period_begin_predicate", order_delivery_period_begin_predicate_);
		GET_CONFIG(privn, n, "order_delivery_period_end_predicate", order_delivery_period_end_predicate_);


		GET_CONFIG(privn, n, "order_instance_type", order_instance_type_);
		GET_CONFIG(privn, n, "order_id_argument", order_id_argument_);
		GET_CONFIG(privn, n, "order_complexity_argument", order_complexity_argument_);
		GET_CONFIG(privn, n, "order_color_argument", order_color_argument_);
		GET_CONFIG(privn, n, "order_gate_argument", order_gate_argument_);

		GET_CONFIG(privn, n, "order_object_prefix", order_object_prefix_);
		GET_CONFIG(privn, n, "gate_object_prefix", gate_object_prefix_);
		GET_CONFIG(privn, n, "order_complexity_object_prefix", order_complexity_object_prefix_);

		GET_CONFIG(privn, n, "rs_ring_value_blue", cfg_rs_ring_value_blue_);
		GET_CONFIG(privn, n, "rs_ring_value_green", cfg_rs_ring_value_green_);
		GET_CONFIG(privn, n, "rs_ring_value_orange", cfg_rs_ring_value_orange_);
		GET_CONFIG(privn, n, "rs_ring_value_yellow", cfg_rs_ring_value_yellow_);


		GET_CONFIG(privn, n, "base_color_value_red", cfg_base_color_value_red_);
		GET_CONFIG(privn, n, "base_color_value_black", cfg_base_color_value_black_);
		GET_CONFIG(privn, n, "base_color_value_silver", cfg_base_color_value_silver_);

		GET_CONFIG(privn, n, "cap_color_value_black", cfg_cap_color_value_black_);
		GET_CONFIG(privn, n, "cap_color_value_grey", cfg_cap_color_value_grey_);



		relevant_predicates_ = {order_complexity_predicate_, order_base_color_predicate_,
		                        order_ring1_color_predicate_, order_ring2_color_predicate_,
		                        order_ring3_color_predicate_, order_cap_color_predicate_,
		                        order_gate_predicate_};

		relevant_predicates_.sort();
		relevant_predicates_.unique();
		relevant_predicates_.remove_if([](const std::string &s) { return s.empty(); });

		relevant_functions_ = {order_delivery_period_begin_predicate_, order_delivery_period_end_predicate_};
		relevant_functions_.sort();
		relevant_functions_.unique();
		relevant_functions_.remove_if([](const std::string &s) { return s.empty(); });


		rs_ring_colors_ = { {rcll_ros_msgs::ProductColor::RING_BLUE, cfg_rs_ring_value_blue_},
		                    {rcll_ros_msgs::ProductColor::RING_GREEN, cfg_rs_ring_value_green_},
		                    {rcll_ros_msgs::ProductColor::RING_ORANGE, cfg_rs_ring_value_orange_},
		                    {rcll_ros_msgs::ProductColor::RING_YELLOW, cfg_rs_ring_value_yellow_} };

		rs_base_colors_ = { {rcll_ros_msgs::ProductColor::BASE_RED, cfg_base_color_value_red_},
		                    {rcll_ros_msgs::ProductColor::BASE_BLACK, cfg_base_color_value_black_},
		                    {rcll_ros_msgs::ProductColor::BASE_SILVER, cfg_base_color_value_silver_} };

		rs_cap_colors_ = {  {rcll_ros_msgs::ProductColor::CAP_BLACK, cfg_cap_color_value_black_},
		                    {rcll_ros_msgs::ProductColor::CAP_GREY, cfg_cap_color_value_grey_} };

		get_predicates();
		get_functions();
	}

	void
	create_svc_update_knowledge()
	{
		svc_update_knowledge_ =
			n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>
			("kcl_rosplan/update_knowledge_base_array", /* persistent */ true);

		ROS_INFO("[RPKB-OrdInfo] Waiting for ROSPlan service update_knowledge_base");
		svc_update_knowledge_.waitForExistence();
	}

	void
	create_svc_current_knowledge()
	{
		svc_current_knowledge_ =
			n.serviceClient<rosplan_knowledge_msgs::GetAttributeService>
			("kcl_rosplan/get_current_knowledge", /* persistent */ true);
		ROS_INFO("[RPKB-OrdInfo] Waiting for ROSPlan service get_current_knowledge");
		svc_current_knowledge_.waitForExistence();
	}

	void
	create_svc_current_instances()
	{
		svc_current_instances_ =
			n.serviceClient<rosplan_knowledge_msgs::GetInstanceService>
			("kcl_rosplan/get_current_instances", /* persistent */ true);
		ROS_INFO("[RPKB-OrdInfo] Waiting for ROSPlan service get_current_instances");
		svc_current_instances_.waitForExistence();
	}

	void
	get_predicates()
	{
		// fetch and store predicate details
		ros::service::waitForService("kcl_rosplan/get_domain_predicate_details",ros::Duration(20));
		ros::ServiceClient pred_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>
			  ("kcl_rosplan/get_domain_predicate_details", /* persistent */ true);
		if (! pred_client.waitForExistence(ros::Duration(20))) {
			ROS_ERROR("[RPKB-OrdInfo] No service provider for get_domain_predicate_details");
			return;
		}

		for (const auto &pn : relevant_predicates_) {
			rosplan_knowledge_msgs::GetDomainPredicateDetailsService pred_srv;
			pred_srv.request.name = pn;
			if(pred_client.call(pred_srv)) {
				std::string pred_str;
				std::for_each(pred_srv.response.predicate.typed_parameters.begin(),
				              pred_srv.response.predicate.typed_parameters.end(),
				              [&pred_str](const auto &kv) { pred_str += " " + kv.key + ":" + kv.value; });
				ROS_INFO("[RPKB-OrdInfo] Relevant predicate: (%s%s)", pn.c_str(), pred_str.c_str());
				predicates_[pn] = pred_srv.response.predicate;
			} else {
				ROS_ERROR("[RPKB-OrdInfo] Failed to get predicate details for %s", pn.c_str());
				return;
			}
		}
	}

	void
	get_functions()
	{
		// fetch and store predicate details
		ros::service::waitForService("kcl_rosplan/get_domain_functions",ros::Duration(20));
		ros::ServiceClient func_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>
			  ("kcl_rosplan/get_domain_functions", /* persistent */ true);
		if (! func_client.waitForExistence(ros::Duration(20))) {
			ROS_ERROR("[RPKB-OrdInfo] No service provider for get_domain_functions");
			return;
		}	

		rosplan_knowledge_msgs::GetDomainAttributeService func_srv;
		
		if (func_client.call(func_srv)) {
			for (const auto &rf : relevant_functions_) {
				bool found_func = false;
				for (const auto &pn : func_srv.response.items) {			
					if (rf == pn.name) {
						found_func = true;
						functions_[rf] = pn;
						ROS_INFO("[RPKB-OrdInfo] Relevant function: %s", pn.name.c_str());
						break;
					}
				}
				if (!found_func) {
					ROS_ERROR("[RPKB-OrdInfo] Failed to get function info for: %s", rf.c_str());
				}
			}
		} else {
			ROS_ERROR("[RPKB-OrdInfo] Failed to get functions info");
		}
	}

	void
	check_unique_predicate(const std::string &predicate_name,
	                       const std::string &idvar_name, const std::string &idvar_value,
	                       const std::map<std::string, std::string> &expected_values,
	                       rosplan_knowledge_msgs::KnowledgeUpdateServiceArray &remsrv,
	                       rosplan_knowledge_msgs::KnowledgeUpdateServiceArray &addsrv)
	{
		if (predicates_.find(predicate_name) != predicates_.end()) {
			rosplan_knowledge_msgs::GetAttributeService srv;
			srv.request.predicate_name = predicate_name;
			if (! svc_current_knowledge_.isValid()) {
				create_svc_current_knowledge();
			}
			if (svc_current_knowledge_.call(srv)) {
				bool not_found_at_all = true;
				for (const auto &a : srv.response.attributes) {
					std::map<std::string, std::string> arguments;
					for (const auto &kv : a.values)  arguments[kv.key] = kv.value;

					if (arguments.find(idvar_name) != arguments.end() &&
					    arguments[idvar_name] == idvar_value)
					{
						not_found_at_all = false;
						if (std::any_of(expected_values.begin(), expected_values.end(),
						                [&arguments](const auto &ev) -> bool
						                { return (arguments.find(ev.first) != arguments.end() &&
						                          arguments[ev.first] != ev.second); }))
							
						{
							ROS_INFO("[RPKB-OrdInfo] Updating '%s' for '%s'", predicate_name.c_str(), idvar_value.c_str());
							rosplan_knowledge_msgs::KnowledgeItem new_a = a;
							std::for_each(expected_values.begin(), expected_values.end(),
							              [&new_a](auto &ev) {
								              for (auto &kv : new_a.values) {
									              if (kv.key == ev.first) {
										              kv.value = ev.second;
										              break;
									              }
								              }
							              });
							remsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::REMOVE_KNOWLEDGE);
							remsrv.request.knowledge.push_back(a);
							addsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_KNOWLEDGE);
							addsrv.request.knowledge.push_back(new_a);
						}
						// we do NOT break here, by this, we also remove any additional
						// fact that matches the idvar and ensure data uniqueness.
					}
				}
				if (not_found_at_all) {
					// It does not exist at all, create
					rosplan_knowledge_msgs::KnowledgeItem new_a;
					new_a.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
					new_a.attribute_name = predicate_name;
					{ // Add ID argument
						diagnostic_msgs::KeyValue kv;
						kv.key = idvar_name; kv.value = idvar_value;
						new_a.values.push_back(kv);
					}
					std::for_each(expected_values.begin(), expected_values.end(),
					              [&new_a](const auto &ev) {
						              diagnostic_msgs::KeyValue kv;
						              kv.key = ev.first; kv.value = ev.second;
						              new_a.values.push_back(kv);
					              });
					addsrv.request.knowledge.push_back(new_a);
					ROS_INFO("[RPKB-OrdInfo] Adding '%s' info for %s", predicate_name.c_str(), idvar_value.c_str());
				}
			} else {
				ROS_ERROR("[RPKB-OrdInfo] Failed to call '%s' for '%s'",
				          svc_current_knowledge_.getService().c_str(), predicate_name.c_str());
			}
		}
	}

	void
	check_function(const std::string &function_name,
	                       const std::string &idvar_name, const std::string &idvar_value,
	                       const std::map<std::string, std::string> &extra_args,
	                       double function_value,
	                       rosplan_knowledge_msgs::KnowledgeUpdateServiceArray &remsrv,
	                       rosplan_knowledge_msgs::KnowledgeUpdateServiceArray &addsrv)
	{
		if (functions_.find(function_name) != functions_.end()) {
			rosplan_knowledge_msgs::GetAttributeService srv;
			srv.request.predicate_name = function_name;
			if (! svc_current_knowledge_.isValid()) {
				create_svc_current_knowledge();
			}
			if (svc_current_knowledge_.call(srv)) {
				bool not_found_at_all = true;
				for (const auto &a : srv.response.attributes) {
					std::map<std::string, std::string> arguments;
					for (const auto &kv : a.values)  arguments[kv.key] = kv.value;

					if (arguments.find(idvar_name) != arguments.end() &&
					    arguments[idvar_name] == idvar_value)
					{
						not_found_at_all = false;
						if (std::any_of(extra_args.begin(), extra_args.end(),
						                [&arguments](const auto &ev) -> bool
						                { return (arguments.find(ev.first) != arguments.end() &&
						                          arguments[ev.first] != ev.second); }))
							
						{
							if (a.function_value != function_value) {
								ROS_INFO("[RPKB-OrdInfo] Updating '%s' for '%s'", function_name.c_str(), idvar_value.c_str());
								rosplan_knowledge_msgs::KnowledgeItem new_a = a;
								std::for_each(extra_args.begin(), extra_args.end(),
									      [&new_a, &function_value](auto &ev) {
										      for (auto &kv : new_a.values) {
											      if (kv.key == ev.first) {
												      kv.value = ev.second;
												      break;
											      }
										      }
										      new_a.function_value = function_value;
									      });
								remsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::REMOVE_KNOWLEDGE);
								remsrv.request.knowledge.push_back(a);
								addsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_KNOWLEDGE);
								addsrv.request.knowledge.push_back(new_a);
							} else {
								ROS_INFO("[RPKB-OrdInfo] No need to update %s", function_name.c_str());
							}
						}
						// we do NOT break here, by this, we also remove any additional
						// fact that matches the idvar and ensure data uniqueness.
					}
				}
				if (not_found_at_all) {
					// It does not exist at all, create
					rosplan_knowledge_msgs::KnowledgeItem new_a;
					new_a.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FUNCTION;
					new_a.attribute_name = function_name;
					{ // Add ID argument
						diagnostic_msgs::KeyValue kv;
						kv.key = idvar_name; kv.value = idvar_value;
						new_a.values.push_back(kv);
					}
					std::for_each(extra_args.begin(), extra_args.end(),
					              [&new_a](const auto &ev) {
						              diagnostic_msgs::KeyValue kv;
						              kv.key = ev.first; kv.value = ev.second;
						              new_a.values.push_back(kv);
					              });
					new_a.function_value = function_value;
					addsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_KNOWLEDGE);
					addsrv.request.knowledge.push_back(new_a);
					ROS_INFO("[RPKB-OrdInfo] Adding '%s' info for %s", function_name.c_str(), idvar_value.c_str());
				}
			} else {
				ROS_ERROR("[RPKB-OrdInfo] Failed to call '%s' for '%s'",
				          svc_current_knowledge_.getService().c_str(), function_name.c_str());
			}
		}
	}

	void
	send_order_predicate_updates(const rcll_ros_msgs::Order &o)
	{
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray remsrv;
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray addsrv;

	
		check_unique_predicate(order_complexity_predicate_, order_id_argument_, order_id_to_name(o.id),
		                       { {order_complexity_argument_, complexity_to_name(o.complexity)} },
		                       remsrv, addsrv);


		check_unique_predicate(order_base_color_predicate_, order_id_argument_, order_id_to_name(o.id),
		                       { {order_color_argument_, rs_base_colors_[o.base_color]} },
		                       remsrv, addsrv);

		check_unique_predicate(order_cap_color_predicate_, order_id_argument_, order_id_to_name(o.id),
		                       { {order_color_argument_, rs_cap_colors_[o.cap_color]} },
		                       remsrv, addsrv);

		check_unique_predicate(order_gate_predicate_, order_id_argument_, order_id_to_name(o.id),
		                       { {order_gate_argument_, delivery_gate_to_name(o.delivery_gate)} },
		                       remsrv, addsrv);

		check_function(order_delivery_period_begin_predicate_, order_id_argument_, order_id_to_name(o.id),
		               {}, o.delivery_period_begin, remsrv, addsrv);
		check_function(order_delivery_period_end_predicate_, order_id_argument_, order_id_to_name(o.id),
		               {}, o.delivery_period_end, remsrv, addsrv);

		if (o.complexity > 0) {
			check_unique_predicate(order_ring1_color_predicate_, order_id_argument_, order_id_to_name(o.id),
		                       { {order_color_argument_, rs_ring_colors_[o.ring_colors[0]]} },
		                       remsrv, addsrv);
		}

		if (o.complexity > 1) {
			check_unique_predicate(order_ring2_color_predicate_, order_id_argument_, order_id_to_name(o.id),
		                       { {order_color_argument_, rs_ring_colors_[o.ring_colors[1]]} },
		                       remsrv, addsrv);
		}


		if (o.complexity > 2) {
			check_unique_predicate(order_ring3_color_predicate_, order_id_argument_, order_id_to_name(o.id),
		                       { {order_color_argument_, rs_ring_colors_[o.ring_colors[2]]} },
		                       remsrv, addsrv);
		}

		if (! remsrv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(remsrv)) {
				ROS_ERROR("[RPKB-OrdInfo] Failed to remove predicates");
			}
		}
		if (! addsrv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(addsrv)) {
				ROS_ERROR("[RPKB-OrdInfo] Failed to add predicates");
				return;
			}
		}
	}


	std::string
	order_id_to_name(int order_id) 
	{
		return order_object_prefix_ + std::to_string(order_id);
	}

	std::string
	complexity_to_name(int complexity) 
	{
		return order_complexity_object_prefix_ + std::to_string(complexity);
	}

	std::string
	delivery_gate_to_name(int delivery_gate) 
	{
		return gate_object_prefix_ + std::to_string(delivery_gate);
	}

	void
	send_order_instance_updates(const std::vector<rcll_ros_msgs::Order> &orders)
	{
		rosplan_knowledge_msgs::GetInstanceService srv;
		srv.request.type_name = order_instance_type_;
		if (! svc_current_instances_.isValid()) {
			create_svc_current_instances();
		}
		if (! svc_current_instances_.call(srv)) {
			ROS_ERROR("[RPKB-OrdInfo] Failed to retrieve current instances of type '%s'",
			          order_instance_type_.c_str());
			return;
		}

		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray addsrv;

		std::vector<std::string> instances = srv.response.instances;
		std::sort(instances.begin(), instances.end());
		for (const rcll_ros_msgs::Order &o : orders) {
			//ROS_WARN("[RPKB-OrdInfo] Got: %d = %s", o.id, order_id_to_name(o.id));
			std::string order_name(order_id_to_name(o.id));
			if (! std::binary_search(instances.begin(), instances.end(), order_name)) {
				rosplan_knowledge_msgs::KnowledgeItem new_i;
				new_i.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
				new_i.instance_type = order_instance_type_;
				new_i.instance_name = order_name;
				addsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_KNOWLEDGE);
				addsrv.request.knowledge.push_back(new_i);
				ROS_INFO("[RPKB-OrdInfo] Adding missing instance '%s - %s'",
				         new_i.instance_name.c_str(), new_i.instance_type.c_str());
			}
		}
		if (! addsrv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(addsrv)) {
				ROS_ERROR("[RPKB-OrdInfo] Failed to add machine instances");
				return;
			}
		}
	}

	void
	order_info_cb(const rcll_ros_msgs::OrderInfo::ConstPtr& msg)
	{
		//ROS_WARN("[RPKB-OrdInfo] Sending updates (conditionally)");
		send_order_instance_updates(msg->orders);
		for (const rcll_ros_msgs::Order &o : msg->orders) {
			send_order_predicate_updates(o);
		}
		last_order_info_ = msg;
	}

 private:
	ros::NodeHandle    n;

	ros::Subscriber    sub_order_info_;
	ros::ServiceClient svc_update_knowledge_;
	ros::ServiceClient svc_current_knowledge_;
	ros::ServiceClient svc_current_instances_;

	std::string order_complexity_predicate_;
	std::string order_base_color_predicate_;
	std::string order_ring1_color_predicate_;
	std::string order_ring2_color_predicate_;
	std::string order_ring3_color_predicate_;
	std::string order_cap_color_predicate_;
	std::string order_gate_predicate_;
	std::string order_delivery_period_begin_predicate_;
	std::string order_delivery_period_end_predicate_;

	std::string order_instance_type_;
	std::string order_id_argument_;
	std::string order_complexity_argument_;
	std::string order_color_argument_;
	std::string order_gate_argument_;



	std::string cfg_rs_ring_value_blue_;
	std::string cfg_rs_ring_value_green_;
	std::string cfg_rs_ring_value_orange_;
	std::string cfg_rs_ring_value_yellow_;

	std::string cfg_base_color_value_red_;
	std::string cfg_base_color_value_black_;
	std::string cfg_base_color_value_silver_;

	std::string cfg_cap_color_value_black_;
	std::string cfg_cap_color_value_grey_;

	std::string order_object_prefix_;
	std::string gate_object_prefix_;
	std::string order_complexity_object_prefix_;



	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates_;
	std::list<std::string> relevant_predicates_;
	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> functions_;
	std::list<std::string> relevant_functions_;

	std::map<int, std::string> rs_ring_colors_;
	std::map<int, std::string> rs_base_colors_;
	std::map<int, std::string> rs_cap_colors_;
	
	rcll_ros_msgs::OrderInfo::ConstPtr last_order_info_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_interface_behaviorengine");

	ros::NodeHandle n;

	ROSPlanKbUpdaterOrderInfo rosplan_kb_updater(n);
	
  ros::spin();
  
	return 0;
}
