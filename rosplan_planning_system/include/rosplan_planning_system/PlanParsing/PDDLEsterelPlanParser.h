#include "PlanParser.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"
#include "diagnostic_msgs/KeyValue.h"

#include <stdlib.h>
#include <string>
#include <fstream>
#include <sstream>
#include <ctime>
#include <algorithm>

#ifndef KCL_pddl_esterel_plan_parser
#define KCL_pddl_esterel_plan_parser

/**
 * This class describes the POPFEsterelPlanParser, which parses the output of popf and generates an Esterel plan.
 */
namespace KCL_rosplan {

	class PDDLEsterelPlanParser: public PlanParser
	{
	private:

		/*---------------------------------*/
		/* ACTION SUPPORT AND INTERFERENCE */
		/*---------------------------------*/

		/**
		 * @returns True if the node satisfies the (possibly negative) condition
		 */
		bool satisfiesPrecondition(rosplan_knowledge_msgs::DomainFormula& condition, const rosplan_dispatch_msgs::EsterelPlanNode& node, bool negative_condition) {
		
			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;

			if(!negative_condition) {
				
				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {
					cit = action_details[node.action.action_id].at_start_add_effects.begin();
					for(; cit!=action_details[node.action.action_id].at_start_add_effects.end(); cit++) {
						if(domainFormulaMatches(condition, *cit)) return true;
					}
				} else if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {
					cit = action_details[node.action.action_id].at_end_add_effects.begin();
					for(; cit!=action_details[node.action.action_id].at_end_add_effects.end(); cit++) {
						if(domainFormulaMatches(condition, *cit)) return true;
					}
				}

			} else {

				if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {
					cit = action_details[node.action.action_id].at_start_del_effects.begin();
					for(; cit!=action_details[node.action.action_id].at_start_del_effects.end(); cit++) {
						if(domainFormulaMatches(condition, *cit)) return true;
					}
				} else if(node.node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {
					cit = action_details[node.action.action_id].at_end_del_effects.begin();
					for(; cit!=action_details[node.action.action_id].at_end_del_effects.end(); cit++) {
						if(domainFormulaMatches(condition, *cit)) return true;
					}
				}
			}

			return false;
		}

		/**
		 * @returns True if the effect satisfies the (possibly negative) condition
		 */
		bool satisfiesPrecondition(rosplan_knowledge_msgs::DomainFormula& condition, const rosplan_knowledge_msgs::KnowledgeItem& effect, bool negative_condition) {
		
			// convert KnowledgeItem to (grounded) DomainFormula
			rosplan_knowledge_msgs::DomainFormula eff;
			eff.name = effect.attribute_name;
			for(int i=0; i<effect.values.size(); i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = effect.values[i].key;
				pair.value = effect.values[i].value;
				eff.typed_parameters.push_back(pair);
			}

			return (negative_condition == effect.is_negative) && domainFormulaMatches(condition, eff);
		}

		/*---------------*/
		/* GRAPH METHODS */
		/*---------------*/

		void makeEdge(int source_node_id, int sink_node_id) {
			// create an ordering that only specifies after
			makeEdge(source_node_id, sink_node_id, 0, std::numeric_limits<double>::max());
		}

		/**
		 * @returns a std::pair describing the upper and lower bounds on the ordering
		 */
		std::pair<double,double> getBounds(rosplan_dispatch_msgs::EsterelPlanNode &a, rosplan_dispatch_msgs::EsterelPlanNode &b) {
			std::set<int> checked_flags;
			return getBounds(a, b, checked_flags);
		}

		std::pair<double,double> getBounds(rosplan_dispatch_msgs::EsterelPlanNode &a, rosplan_dispatch_msgs::EsterelPlanNode &b, std::set<int> &checked_flags) {

			// a node happens when it happens
			if(a.node_id==b.node_id) 
				return std::make_pair<double,double>(0,0);

			// potentially no bounds
			std::pair<double,double> bounds = std::make_pair<double,double>(-std::numeric_limits<double>::max(),std::numeric_limits<double>::max());

			// else check all nodes ordered after a
			for(size_t i=0; i<a.edges_out.size(); i++) {
				int edge_id = a.edges_out[i];
				for(size_t j=0; j<last_plan.edges[edge_id].sink_ids.size(); j++) {
					int node_id = last_plan.edges[edge_id].sink_ids[j];
					// get bounds from child
					std::pair<double,double> newbounds = getBounds(last_plan.nodes[node_id], b, checked_flags);
					// update bounds
					if(bounds.first < newbounds.first + last_plan.edges[edge_id].duration_lower_bound)
						bounds.first = newbounds.first + last_plan.edges[edge_id].duration_lower_bound;
					if(bounds.second > newbounds.second + last_plan.edges[edge_id].duration_upper_bound)
						bounds.second = newbounds.second + last_plan.edges[edge_id].duration_upper_bound;
				}
			}

			return bounds;
		}

		/*------------------------*/
		/* DOMAIN FORMULA METHODS */
		/*------------------------*/

		/**
		 * Grounds the given vector of domain formulae. Resulting in e.g., (robot_at ?wp - wp01)
		 * @param formula  The domain formula to be grounded, e.g., (robot_at ?wp - waypoint)
		 * @param opParams An ordered KeyValue vector of operator parameters, e.g., (?r - robot ?wp - waypoint)
		 * @param params   An ordered vector of parameter objects, e.g., [robot01, wp01]
		 */
		void groundFormula(std::vector<rosplan_knowledge_msgs::DomainFormula> &formula, std::vector<diagnostic_msgs::KeyValue> &opParams, std::vector<std::string> &params) {

			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;
			cit = formula.begin();
			for(; cit!=formula.end(); cit++) {
				// for each predicate parameter
				for(int j=0;j<cit->typed_parameters.size();j++) {
					bool found = false;
					// for each operator parameter
					for(size_t i=0; i<opParams.size(); i++) {
						// operator and predicate parmater labels match
						if(opParams[i].key == cit->typed_parameters[j].key) {
							// set predicate parameter to be: label -> value
							cit->typed_parameters[j].value = params[i];
							found = true;
						}
					}
					// if not found, then label must be a constant
					if(!found) {
						// set predicate parameter to be: value -> value
						// key could be fetched by "get_predicate_details" service, but is not required
						cit->typed_parameters[j].value = cit->typed_parameters[j].key;
					}
				}
			}
		}

		/**
		 * @returns True if the two domain formula parameters match completely.
		 */
		bool domainFormulaMatches(rosplan_knowledge_msgs::DomainFormula& a, rosplan_knowledge_msgs::DomainFormula& b) {
			if(b.name != a.name) return false;
			if(b.typed_parameters.size() != a.typed_parameters.size()) return false;
			for(size_t i=0; i<a.typed_parameters.size(); i++) {
				if(a.typed_parameters[i].value != b.typed_parameters[i].value) {
					return false;
				}
			}
			return true;
		}

		/* plan description in Esterel */
		rosplan_dispatch_msgs::EsterelPlan last_plan;
		std::map<int,rosplan_knowledge_msgs::DomainOperator> action_details;

		/* TILs */
		ros::ServiceClient get_predicate_client;
		ros::ServiceClient get_knowledge_client;
		std::multimap<double, rosplan_knowledge_msgs::KnowledgeItem> til_list;

	protected:

		/* post process plan */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params);
		void fetchTILs();
		void createGraph();
		bool addInterferenceEdges(std::multimap<double,int> &node_map, std::multimap<double,int>::iterator &current_node);
		void makeEdge(int source_node_id, int sink_node_id, double lower_bound, double upper_bound);
		bool addConditionEdge(
				std::multimap<double,int> &node_map, std::multimap<double,int>::iterator &current_node,
				rosplan_knowledge_msgs::DomainFormula &condition, bool negative_condition, bool overall_condition);

		/* virtual methods */
		void reset();
		void preparePlan();
		void publishPlan();

	public:

		/* constructor */
		PDDLEsterelPlanParser(ros::NodeHandle &nh);
		~PDDLEsterelPlanParser();

		/* ROS interface */
		ros::Publisher plan_publisher;
	};
} // close namespace

#endif
