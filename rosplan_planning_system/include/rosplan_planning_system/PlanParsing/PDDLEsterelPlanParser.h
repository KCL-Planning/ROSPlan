#include "PlanParser.h"

#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/EsterelPlan.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"
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

		/*---------------*/
		/* GRAPH METHODS */
		/*---------------*/

		void makeEdge(int source_node_id, int sink_node_id) {

			// don't make edge if ordering already exists
			if(isOrdered(last_plan.nodes[source_node_id], last_plan.nodes[sink_node_id]))
				return;

			// create and add edge
			rosplan_dispatch_msgs::EsterelPlanEdge newEdge;
			newEdge.edge_id = last_plan.edges.size();
			std::stringstream ss;
			ss << "edge" << "_" << newEdge.edge_id;
			newEdge.edge_name = ss.str();
			newEdge.signal_type = 0;
			newEdge.source_ids.push_back(source_node_id);
			newEdge.sink_ids.push_back(sink_node_id);

			last_plan.edges.push_back(newEdge);

			last_plan.nodes[source_node_id].edges_out.push_back(newEdge.edge_id);
			last_plan.nodes[sink_node_id].edges_in.push_back(newEdge.edge_id);
		}

		/**
		 * @returns True ifF node A is ordered before node B by existing edges
		 */
		bool isOrdered(rosplan_dispatch_msgs::EsterelPlanNode &a, rosplan_dispatch_msgs::EsterelPlanNode &b) {
			std::set<int> checked_flags;
			return isOrdered(a, b, checked_flags);
		}

		bool isOrdered(rosplan_dispatch_msgs::EsterelPlanNode &a, rosplan_dispatch_msgs::EsterelPlanNode &b, std::set<int> checked_flags) {

			if(a.node_id==b.node_id) return true;

			checked_flags.insert(a.node_id);

			// else check all nodes ordered after a
			for(size_t i=0; i<a.edges_out.size(); i++) {
				int edge_id = a.edges_out[i];
				for(size_t j=0; j<last_plan.edges[edge_id].sink_ids.size(); j++) {
					int node_id = last_plan.edges[edge_id].sink_ids[j];

					if(checked_flags.count(node_id) == 0) {
						if(isOrdered(last_plan.nodes[node_id], b)) {
							return true;
						}
					}
				}
			}
			return false;
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
			for(size_t i=0; i<opParams.size(); i++) {
				std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;
				cit = formula.begin();
				for(; cit!=formula.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
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

	protected:

		/* post process plan */
		void processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params);
		void createGraph();
		bool addConditionEdge(std::map<double,int> &node_map, std::map<double,int>::iterator &current_node, rosplan_knowledge_msgs::DomainFormula &condition, bool negative_condition);
		void addInterferenceEdges(std::map<double,int> &node_map, std::map<double,int>::iterator &current_node);

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
