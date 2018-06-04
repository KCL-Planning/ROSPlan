/***************************************************************************
 *  rosplan_interface_rcllrefbox.cpp - Referee box actions
 *
 *  Created: Wed Feb 15 21:37:18 2017
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

#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_dispatch_msgs/ActionDispatch.h>
#include <rosplan_knowledge_msgs/DomainFormula.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorService.h>
#include <rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h>
#include <rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h>
#include <diagnostic_msgs/KeyValue.h>
#include <rcll_ros_msgs/SendPrepareMachine.h>
#include <rcll_ros_msgs/ProductColor.h>

#include <map>
#include <string>

typedef enum {
	ACTION_ENABLED,
	ACTION_ACHIEVED,
	ACTION_FAILED
} ActionStatus;

static std::map<ActionStatus, std::string> ActionStatus2String
	{ {ACTION_ENABLED, "action enabled"},
	  {ACTION_ACHIEVED, "action achieved"},
	  {ACTION_FAILED, "action failed"} };

#define GET_CONFIG(privn, n, path, var)	  \
	if (! privn.getParam(path, var)) {      \
		n.getParam(path, var);                \
	}

std::string str_toupper(const std::string &s)
{
	std::string rv(s);
	std::transform(s.begin(), s.end(), rv.begin(), ::toupper);
	return rv;
}

class ROSPlanInterfaceRCLLRefBox {
 public:
	ROSPlanInterfaceRCLLRefBox(ros::NodeHandle &n)
		: n(n)
	{
		sub_action_dispatch_ = n.subscribe("kcl_rosplan/action_dispatch", 10,
		                                   &ROSPlanInterfaceRCLLRefBox::action_dispatch_cb, this);
		pub_action_feedback_ =
			n.advertise<rosplan_dispatch_msgs::ActionFeedback>("kcl_rosplan/action_feedback", 10, true);

		create_svc_update_knowledge();
		create_svc_refbox_prepare();

		ros::NodeHandle privn("~");
		GET_CONFIG(privn, n, "opname/prepare_bs", cfg_opname_prepare_bs_);
		GET_CONFIG(privn, n, "opname/prepare_ds", cfg_opname_prepare_ds_);
		GET_CONFIG(privn, n, "opname/prepare_cs", cfg_opname_prepare_cs_);
		GET_CONFIG(privn, n, "opname/prepare_rs", cfg_opname_prepare_rs_);
		GET_CONFIG(privn, n, "preparg/mps_var", cfg_preparg_mps_var_);
		GET_CONFIG(privn, n, "preparg/bs_side_var", cfg_preparg_bs_side_var_);
		GET_CONFIG(privn, n, "preparg/bs_side_input", cfg_preparg_bs_side_input_);
		GET_CONFIG(privn, n, "preparg/bs_side_output", cfg_preparg_bs_side_output_);
		GET_CONFIG(privn, n, "preparg/bs_base_var", cfg_preparg_bs_base_var_);
		GET_CONFIG(privn, n, "preparg/bs_base_red", cfg_preparg_bs_base_red_);
		GET_CONFIG(privn, n, "preparg/bs_base_black", cfg_preparg_bs_base_black_);
		GET_CONFIG(privn, n, "preparg/bs_base_silver", cfg_preparg_bs_base_silver_);
		GET_CONFIG(privn, n, "preparg/ds_gate_var", cfg_preparg_ds_gate_var_);
		GET_CONFIG(privn, n, "preparg/ds_gate_1", cfg_preparg_ds_gate_1_);
		GET_CONFIG(privn, n, "preparg/ds_gate_2", cfg_preparg_ds_gate_2_);
		GET_CONFIG(privn, n, "preparg/ds_gate_3", cfg_preparg_ds_gate_3_);
		GET_CONFIG(privn, n, "preparg/cs_op_var", cfg_preparg_cs_op_var_);
		GET_CONFIG(privn, n, "preparg/cs_retrieve", cfg_preparg_cs_retrieve_);
		GET_CONFIG(privn, n, "preparg/cs_mount", cfg_preparg_cs_mount_);
		GET_CONFIG(privn, n, "preparg/rs_ring_var", cfg_preparg_rs_ring_var_);
		GET_CONFIG(privn, n, "preparg/rs_ring_blue", cfg_preparg_rs_blue_);
		GET_CONFIG(privn, n, "preparg/rs_ring_green", cfg_preparg_rs_green_);
		GET_CONFIG(privn, n, "preparg/rs_ring_orange", cfg_preparg_rs_orange_);
		GET_CONFIG(privn, n, "preparg/rs_ring_yellow", cfg_preparg_rs_yellow_);

		if (! privn.getParam("succeed_actions", cfg_succeed_actions_)) {
			n.getParam("succeed_actions", cfg_succeed_actions_);
		}
		std::sort(cfg_succeed_actions_.begin(), cfg_succeed_actions_.end());

		if (! privn.getParam("ignored_effect_predicates", cfg_igneffect_preds_)) {
			n.getParam("ignored_effect_predicates", cfg_igneffect_preds_);
		}
		if (! cfg_igneffect_preds_.empty()) {
			std::sort(cfg_igneffect_preds_.begin(), cfg_igneffect_preds_.end());
			
			std::string pred_str;
			std::for_each(cfg_igneffect_preds_.begin(), cfg_igneffect_preds_.end(),
			              [&pred_str](const auto &p) { pred_str += " " + p; });
			ROS_INFO("[RPI-RefBox] Ignored effect predicates:%s", pred_str.c_str());
		}

		relevant_actions_.push_back(cfg_opname_prepare_bs_);
		relevant_actions_.push_back(cfg_opname_prepare_ds_);
		relevant_actions_.push_back(cfg_opname_prepare_cs_);
		relevant_actions_.push_back(cfg_opname_prepare_rs_);
		relevant_actions_.sort();

		get_action_specs();

		if (! cfg_succeed_actions_.empty()) {
			std::string act_str;
			bool first = true;
			std::for_each(cfg_succeed_actions_.begin(), cfg_succeed_actions_.end(),
			              [&act_str, &first](const auto &a) {
				              if (first) first = false; else act_str += ",";
				              act_str += a;
			              });
			ROS_INFO("[RPI-RefBox] Always succeeding actions: %s", act_str.c_str());
		}
	}

	void
	create_svc_update_knowledge()
	{
		svc_update_knowledge_ =
			n.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>
			("kcl_rosplan/update_knowledge_base_array", /* persistent */ true);

		ROS_INFO("[RPI-RefBox] Waiting for ROSPlan service update_knowledge_base");
		svc_update_knowledge_.waitForExistence();
	}

	void
	create_svc_refbox_prepare()
	{
		svc_refbox_prepare_ =
			n.serviceClient<rcll_ros_msgs::SendPrepareMachine>("rcll/send_prepare_machine",
			                                                   /* persistent */ true);

		ROS_INFO("[RPI-RefBox] Waiting for RCLL refbox service send_prepare_machine");
		svc_refbox_prepare_.waitForExistence();
	}

	void
	get_action_spec(const std::string &name)
	{
		ros::ServiceClient opdetail_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>
			  ("kcl_rosplan/get_domain_operator_details");
		//ROS_INFO("[RPI-RefBox] Waiting for ROSPlan service get_domain_operator_details");
		if (! opdetail_client.waitForExistence(ros::Duration(10))) {
			ROS_ERROR("[RPI-RefBox] Could not discover get_domain_operator_details service "
			          "(for action spec '%s')", name.c_str());
			return;
		}
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = name;
		if (opdetail_client.call(srv)) {
			bool has_robot_var = false;

			//ROS_INFO("[RPI-RefBox] Parsing get_domain_operator_details response");
			std::set<std::string> reqp;
			for (const auto &p : srv.response.op.formula.typed_parameters) {
				reqp.insert(p.key);
			}

			specs_[name] = { srv.response.op.formula, srv.response.op, reqp };
		} else {
			ROS_ERROR("[RPI-RefBox] Could not get spec for operator %s", name.c_str());
			return;
		}
	}

	void
	get_action_specs()
	{
		ros::ServiceClient oplist_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorService>
			  ("kcl_rosplan/get_domain_operators");
		if (! oplist_client.waitForExistence(ros::Duration(120))) {
			ROS_ERROR("[RPI-RefBox] Could not retrieve action specs from ROSPlan");
			return;
		}
		
		rosplan_knowledge_msgs::GetDomainOperatorService oplist_srv;
		if (oplist_client.call(oplist_srv)) {
			for (const auto &op : oplist_srv.response.operators) {
				if (std::binary_search(relevant_actions_.begin(), relevant_actions_.end(), op.name) ||
				    std::binary_search(cfg_succeed_actions_.begin(), cfg_succeed_actions_.end(), op.name))
				{
					ROS_INFO("[RPI-RefBox] Retrieving action spec for %s", op.name.c_str());
					get_action_spec(op.name);
				} else {
					ROS_INFO("[RPI-RefBox] Ignoring irrelevant action '%s'", op.name.c_str());
				}
			}
		} else {
			ROS_ERROR("[RPI-RefBox] Failed to get list of operators");
		}

		get_predicates();
	}
	
	void collect_predicates(std::set<std::string> &predicate_names,
	                        const std::vector<rosplan_knowledge_msgs::DomainFormula> &df)
	{
		std::for_each(df.begin(), df.end(),
		              [&predicate_names](auto &f) { predicate_names.insert(f.name); });
	}

	void
	get_predicates()
	{
		std::set<std::string> predicate_names;
		for (const auto &sp : specs_) {
			const RPActionSpec &s = sp.second;
			collect_predicates(predicate_names, s.op.at_start_add_effects);
			collect_predicates(predicate_names, s.op.at_start_del_effects);
			collect_predicates(predicate_names, s.op.at_end_add_effects);
			collect_predicates(predicate_names, s.op.at_end_del_effects);
			collect_predicates(predicate_names, s.op.at_start_simple_condition);
			collect_predicates(predicate_names, s.op.over_all_simple_condition);
			collect_predicates(predicate_names, s.op.at_end_simple_condition);
			collect_predicates(predicate_names, s.op.at_start_neg_condition);
			collect_predicates(predicate_names, s.op.over_all_neg_condition);
			collect_predicates(predicate_names, s.op.at_end_neg_condition);
		}

		// fetch and store predicate details
		ros::service::waitForService("kcl_rosplan/get_domain_predicate_details",ros::Duration(20));
		ros::ServiceClient pred_client =
			n.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>
			  ("kcl_rosplan/get_domain_predicate_details", /* persistent */ true);
		if (! pred_client.waitForExistence(ros::Duration(20))) {
			ROS_ERROR("[RPI-RefBox] No service provider for get_domain_predicate_details");
			return;
		}

		for (const auto &pn : predicate_names) {
			if (std::binary_search(cfg_igneffect_preds_.begin(), cfg_igneffect_preds_.end(), pn)) {
				// Ignore predicates which we do not update
				continue;
			}
			// equality is a built-in predicate
			if (pn == "=") continue;

			rosplan_knowledge_msgs::GetDomainPredicateDetailsService pred_srv;
			pred_srv.request.name = pn;
			if(pred_client.call(pred_srv)) {
				std::string pred_str;
				std::for_each(pred_srv.response.predicate.typed_parameters.begin(),
				              pred_srv.response.predicate.typed_parameters.end(),
				              [&pred_str](const auto &kv) { pred_str += " " + kv.key + ":" + kv.value; });
				ROS_INFO("[RPI-RefBox] Relevant predicate: (%s%s)", pn.c_str(), pred_str.c_str());
				predicates_[pn] = pred_srv.response.predicate;
			} else {
				ROS_ERROR("[RPI-RefBox] Failed to get predicate details for %s", pn.c_str());
				return;
			}
		}
	}

	void
	send_action_fb(int id, ActionStatus status)
	{
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = id;
		fb.status = ActionStatus2String[status];
		pub_action_feedback_.publish(fb);
	}
	
	void
	action_dispatch_cb(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
	{
		const std::string &name(msg->name);

		if (specs_.find(name) == specs_.end()) {
			ROS_INFO("[RPI-RefBox] Unknown or ignored action %s called, ignoring", name.c_str());
			return;
		}

		std::set<std::string> params;
		std::for_each(msg->parameters.begin(), msg->parameters.end(),
		              [&params](const auto &kv) { params.insert(kv.key); });

		std::vector<std::string> diff;
		std::set_difference(specs_[name].required_params.begin(), specs_[name].required_params.end(),
		                    params.begin(), params.end(), std::inserter(diff, diff.begin()));

		if (! diff.empty()) {
			std::string diff_s;
			std::for_each(diff.begin(), diff.end(), [&diff_s](const auto &s) { diff_s += " " + s; });
			ROS_WARN("[RPI-RefBox] Invalid call to %s (invalid or missing args %s), failing",
			         name.c_str(), diff_s.c_str());
			send_action_fb(msg->action_id, ACTION_FAILED);
			return;
		}

		std::map<std::string, std::string>    bound_params;
		get_bound_params(*msg, bound_params);

		if (bound_params.find(cfg_preparg_mps_var_) == bound_params.end()) {
			ROS_ERROR("[RPI-RefBox] Action '%s' does not have MPS argument '%s'",
			          name.c_str(), cfg_preparg_mps_var_.c_str());
			
			send_action_fb(msg->action_id, ACTION_FAILED);
			return;
		}

		if (name == cfg_opname_prepare_bs_ || name == cfg_opname_prepare_cs_ ||
		    name == cfg_opname_prepare_ds_ || name == cfg_opname_prepare_rs_)
		{
			rcll_ros_msgs::SendPrepareMachine srv;
			srv.request.machine = str_toupper(bound_params[cfg_preparg_mps_var_]);
			srv.request.wait = true;

			if (name == cfg_opname_prepare_bs_) {
				if (bound_params.find(cfg_preparg_bs_side_var_) == bound_params.end()) {
					ROS_ERROR("[RPI-RefBox] Action '%s' to prepare BS does not have side argument '%s'",
					          name.c_str(), cfg_preparg_bs_side_var_.c_str());
					send_action_fb(msg->action_id, ACTION_FAILED);
					return;
				}
				if (bound_params.find(cfg_preparg_bs_base_var_) == bound_params.end()) {
					ROS_ERROR("[RPI-RefBox] Action '%s' to prepare BS does not have base color argument '%s'",
					          name.c_str(), cfg_preparg_bs_base_var_.c_str());
					send_action_fb(msg->action_id, ACTION_FAILED);
					return;
				}

				const std::string &bs_side = bound_params[cfg_preparg_bs_side_var_];
				if (bs_side == cfg_preparg_bs_side_input_) {
					srv.request.bs_side = rcll_ros_msgs::SendPrepareMachine::Request::BS_SIDE_INPUT;
				} else if (bs_side == cfg_preparg_bs_side_output_) {
					srv.request.bs_side = rcll_ros_msgs::SendPrepareMachine::Request::BS_SIDE_OUTPUT;
				} else {
					ROS_ERROR("[RPI-RefBox] Action '%s' to prepare BS invalid side argument '%s', must be '%s' or '%s'",
					          name.c_str(), bs_side.c_str(),
					          cfg_preparg_bs_side_input_.c_str(), cfg_preparg_bs_side_output_.c_str());
					send_action_fb(msg->action_id, ACTION_FAILED);
					return;
				}

				const std::string &bs_base_color = bound_params[cfg_preparg_bs_base_var_];
				if (bs_base_color == cfg_preparg_bs_base_red_) {
					srv.request.bs_base_color = rcll_ros_msgs::ProductColor::BASE_RED;
				} else if (bs_base_color == cfg_preparg_bs_base_black_) {
					srv.request.bs_base_color = rcll_ros_msgs::ProductColor::BASE_BLACK;
				} else if (bs_base_color == cfg_preparg_bs_base_silver_) {
					srv.request.bs_base_color = rcll_ros_msgs::ProductColor::BASE_SILVER;
				} else {
					ROS_ERROR("[RPI-RefBox] Action '%s' to prepare BS invalid base color argument '%s', must be '%s', '%s', or '%s'",
					          name.c_str(), bs_side.c_str(), cfg_preparg_bs_base_red_.c_str(),
					          cfg_preparg_bs_base_black_.c_str(), cfg_preparg_bs_base_silver_.c_str());
					send_action_fb(msg->action_id, ACTION_FAILED);
					return;
				}

			} else if (name == cfg_opname_prepare_ds_) {

				const std::string &ds_gate = bound_params[cfg_preparg_ds_gate_var_];
				if (ds_gate == cfg_preparg_ds_gate_1_) {
					srv.request.ds_gate = 1;
				} else if (ds_gate == cfg_preparg_ds_gate_2_) {
					srv.request.ds_gate = 2;
				} else if (ds_gate == cfg_preparg_ds_gate_3_) {
					srv.request.ds_gate = 3;
				} else {
					ROS_ERROR("[RPI-RefBox] Action '%s' to prepare DS invalid gate argument '%s', must be '%s', '%s', or '%s'",
					          name.c_str(), ds_gate.c_str(), cfg_preparg_ds_gate_1_.c_str(),
					          cfg_preparg_ds_gate_2_.c_str(), cfg_preparg_ds_gate_3_.c_str());
					send_action_fb(msg->action_id, ACTION_FAILED);
					return;
				}
				
			} else if (name == cfg_opname_prepare_cs_) {

				const std::string &cs_operation = bound_params[cfg_preparg_cs_op_var_];
				if (cs_operation == cfg_preparg_cs_retrieve_) {
					srv.request.cs_operation = rcll_ros_msgs::SendPrepareMachine::Request::CS_OP_RETRIEVE_CAP;
				} else if (cs_operation == cfg_preparg_cs_mount_) {
					srv.request.cs_operation = rcll_ros_msgs::SendPrepareMachine::Request::CS_OP_MOUNT_CAP;
				} else {
					ROS_ERROR("[RPI-RefBox] Action '%s' to prepare CS invalid operation argument '%s', must be '%s' or '%s'",
					          name.c_str(), cs_operation.c_str(), cfg_preparg_cs_retrieve_.c_str(),
					          cfg_preparg_cs_mount_.c_str());
					send_action_fb(msg->action_id, ACTION_FAILED);
					return;
				}
				
			} else if (name == cfg_opname_prepare_rs_) {

				const std::string &rs_ring_color = bound_params[cfg_preparg_rs_ring_var_];
				if (rs_ring_color == cfg_preparg_rs_blue_) {
					srv.request.rs_ring_color = rcll_ros_msgs::ProductColor::RING_BLUE;
				} else if (rs_ring_color == cfg_preparg_rs_green_) {
					srv.request.rs_ring_color = rcll_ros_msgs::ProductColor::RING_GREEN;
				} else if (rs_ring_color == cfg_preparg_rs_orange_) {
					srv.request.rs_ring_color = rcll_ros_msgs::ProductColor::RING_ORANGE;
				} else if (rs_ring_color == cfg_preparg_rs_yellow_) {
					srv.request.rs_ring_color = rcll_ros_msgs::ProductColor::RING_YELLOW;
				} else {
					ROS_ERROR("[RPI-RefBox] Action '%s' to prepare RS invalid ring color argument '%s', "
					          "must be '%s', '%s', '%s', or '%s'",
					          name.c_str(), rs_ring_color.c_str(), cfg_preparg_rs_blue_.c_str(),
					          cfg_preparg_rs_green_.c_str(), cfg_preparg_rs_orange_.c_str(),
					          cfg_preparg_rs_yellow_.c_str());
					send_action_fb(msg->action_id, ACTION_FAILED);
					return;
				}

			} else {
				ROS_ERROR("[RPI-RefBox] WTF!?");
				send_action_fb(msg->action_id, ACTION_FAILED);
				return;
			}

			ROS_INFO("[RPI-RefBox] Calling %s %s %i %i %u %i %i",
			         svc_refbox_prepare_.getService().c_str(),
			         srv.request.machine.c_str(),
			         srv.request.bs_side, srv.request.bs_base_color,
			         srv.request.ds_gate, srv.request.rs_ring_color, srv.request.cs_operation);
			if (! svc_refbox_prepare_.isValid()) {
				create_svc_refbox_prepare();
			}
			if( ! svc_refbox_prepare_.call(srv)) {
				ROS_WARN("[RPI-RefBox] Failed to call SendPrepare for '%s'", name.c_str());
				send_action_fb(msg->action_id, ACTION_FAILED);
				return;
			} else if (! srv.response.ok) {
				ROS_WARN("[RPI-RefBox] Execution of SendPrepare for '%s' failed: %s",
				         name.c_str(), srv.response.error_msg.c_str());
				send_action_fb(msg->action_id, ACTION_FAILED);
				return;
			} else {
				ROS_INFO("[RPI-RefBox] Execution of SendPrepare for '%s' succeeded", name.c_str());
			}

		} else if (std::binary_search(cfg_succeed_actions_.begin(), cfg_succeed_actions_.end(), name)) {
			ROS_INFO("[RPI-RefBox] Always succeeding action '%s' called", name.c_str());

		} else {
			ROS_ERROR("[RPI-RefBox] Received action for unhandled op '%s'", name.c_str());
			send_action_fb(msg->action_id, ACTION_FAILED);
			return;
		}

		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray remsrv;
		rosplan_knowledge_msgs::KnowledgeUpdateServiceArray addsrv;


		proc_predicate_updates(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::REMOVE_KNOWLEDGE,
		                       specs_[name].op.at_start_del_effects, bound_params, remsrv, addsrv);
		proc_predicate_updates(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE,
		                       specs_[name].op.at_start_add_effects, bound_params, remsrv, addsrv);
		proc_predicate_updates(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::REMOVE_KNOWLEDGE,
		                       specs_[name].op.at_end_del_effects, bound_params, remsrv, addsrv);
		proc_predicate_updates(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE,
		                       specs_[name].op.at_end_add_effects, bound_params, remsrv, addsrv);

		if (! remsrv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(remsrv)) {
				ROS_ERROR("[RPI-RefBox] Failed to remove predicates");
				send_action_fb(msg->action_id, ACTION_FAILED);
				return;
			}
		}
		if (! addsrv.request.knowledge.empty()) {
			if (! svc_update_knowledge_.isValid()) {
				create_svc_update_knowledge();
			}
			if( ! svc_update_knowledge_.call(addsrv)) {
				ROS_ERROR("[RPI-RefBox] Failed to add predicates");
				send_action_fb(msg->action_id, ACTION_FAILED);
				return;
			}
		}

		send_action_fb(msg->action_id, ACTION_ACHIEVED);
	}

	void
	get_bound_params(const rosplan_dispatch_msgs::ActionDispatch &msg,
	                 std::map<std::string, std::string> &bound_params)
	{
		bound_params.clear();
		std::for_each(specs_[msg.name].params.typed_parameters.begin(),
		              specs_[msg.name].params.typed_parameters.end(),
		              [&bound_params, &msg](const auto &kv) {
			              for (const auto &mp : msg.parameters) {
				              if (kv.key == mp.key) {
					              bound_params[mp.key] = mp.value;
					              break;
				              }
			              }});
	}
	
	void
	proc_predicate_updates(int op,
	                       const std::vector<rosplan_knowledge_msgs::DomainFormula> &dfv,
	                       std::map<std::string, std::string> &bound_params,
	                       rosplan_knowledge_msgs::KnowledgeUpdateServiceArray &remsrv,
	                       rosplan_knowledge_msgs::KnowledgeUpdateServiceArray &addsrv)
	{
		for (const auto &df : dfv) {
			if (std::binary_search(cfg_igneffect_preds_.begin(), cfg_igneffect_preds_.end(), df.name)) {
				// Ignore this predicate and continue with the next
				ROS_INFO("[RPI-RefBox] Ignoring effect predicate '%s'", df.name.c_str());
				continue;
			}

			rosplan_knowledge_msgs::KnowledgeItem item;
			item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			item.attribute_name = df.name;

			if (predicates_.find(df.name) == predicates_.end()) {
				ROS_ERROR("[RPI-RefBox] Unknown predicate %s, cannot update", df.name.c_str());
				continue;
			}
			if (predicates_[df.name].typed_parameters.size() != df.typed_parameters.size()) {
				ROS_ERROR("[RPI-RefBox] Inconsistent typed parameters for %s", df.name.c_str());
				continue;
			}

			for(size_t j=0; j < df.typed_parameters.size(); ++j) {
				diagnostic_msgs::KeyValue pair;
				pair.key = predicates_[df.name].typed_parameters[j].key;
				if (bound_params.find(df.typed_parameters[j].key) == bound_params.end()) {
					// this must be a constant then
					pair.value = df.typed_parameters[j].key;
				} else {
					pair.value = bound_params[df.typed_parameters[j].key];
				}
				item.values.push_back(pair);
			}

			std::string param_str;
			std::for_each(item.values.begin(), item.values.end(),
			              [&param_str](const auto &kv) { param_str += " " + kv.key + "=" + kv.value; });

			if (op == rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request::ADD_KNOWLEDGE) {
				ROS_INFO("[RPI-RefBox] Asserting (%s%s)",
				         item.attribute_name.c_str(), param_str.c_str());
				addsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::ADD_KNOWLEDGE);
				addsrv.request.knowledge.push_back(item);
			} else {
				ROS_INFO("[RPI-RefBox] Retracting (%s%s)",
				         item.attribute_name.c_str(), param_str.c_str());
				remsrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateServiceArrayRequest::REMOVE_KNOWLEDGE);
				remsrv.request.knowledge.push_back(item);
			}
		}
	}
	
 private:
	ros::NodeHandle    n;

	ros::Subscriber    sub_action_dispatch_;
	ros::Publisher     pub_action_feedback_;
	ros::ServiceClient svc_update_knowledge_;

	ros::ServiceClient svc_refbox_prepare_;

	struct RPActionSpec {
		rosplan_knowledge_msgs::DomainFormula  params;
		rosplan_knowledge_msgs::DomainOperator op;
		std::set<std::string> required_params;
	};
	std::map<std::string, RPActionSpec> specs_;
	std::map<std::string, rosplan_knowledge_msgs::DomainFormula> predicates_;

	std::string cfg_opname_prepare_bs_;
	std::string cfg_opname_prepare_ds_;
	std::string cfg_opname_prepare_cs_;
	std::string cfg_opname_prepare_rs_;
	std::string cfg_preparg_mps_var_;
	std::string cfg_preparg_bs_side_var_;
	std::string cfg_preparg_bs_side_input_;
	std::string cfg_preparg_bs_side_output_;
	std::string cfg_preparg_bs_base_var_;
	std::string cfg_preparg_bs_base_red_;
	std::string cfg_preparg_bs_base_black_;
	std::string cfg_preparg_bs_base_silver_;
	std::string cfg_preparg_ds_gate_var_;
	std::string cfg_preparg_ds_gate_1_;
	std::string cfg_preparg_ds_gate_2_;
	std::string cfg_preparg_ds_gate_3_;
	std::string cfg_preparg_cs_op_var_;
	std::string cfg_preparg_cs_retrieve_;
	std::string cfg_preparg_cs_mount_;
	std::string cfg_preparg_rs_ring_var_;
	std::string cfg_preparg_rs_blue_;
	std::string cfg_preparg_rs_green_;
	std::string cfg_preparg_rs_orange_;
	std::string cfg_preparg_rs_yellow_;

	std::vector<std::string> cfg_igneffect_preds_;
	std::vector<std::string> cfg_succeed_actions_;

	std::list<std::string> relevant_actions_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "rosplan_interface_behaviorengine");

	ros::NodeHandle n;

	ROSPlanInterfaceRCLLRefBox rosplan_rcll_refbox(n);
	
  ros::spin();
  
	return 0;
}
