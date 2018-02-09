#include "rosplan_planning_system/PlanParsing/PDDLEsterelPlanParser.h"

/* CHANGE CHANGE */

/* implementation of rosplan_planning_system::PDDLEsterelPlanParser.h */
namespace KCL_rosplan {

	/* constructor */
	PDDLEsterelPlanParser::PDDLEsterelPlanParser(ros::NodeHandle &nh)
	{
		node_handle = &nh;

		// publishing parsed plan
		std::string planTopic = "complete_plan";
		node_handle->getParam("plan_topic", planTopic);
		plan_publisher = node_handle->advertise<rosplan_dispatch_msgs::EsterelPlan>(planTopic, 1, true);
	}

	PDDLEsterelPlanParser::~PDDLEsterelPlanParser()
	{

	}

	void PDDLEsterelPlanParser::reset() {
		action_list.clear();
		last_plan.nodes.clear();
		last_plan.edges.clear();
	}

	void PDDLEsterelPlanParser::publishPlan() {
		plan_publisher.publish(last_plan);
	}

	/*----------------------*/
	/* Post processing plan */
	/*----------------------*/

	/**
	 * parses standard PDDL output, generating an esterel plan in graph form.
	 */
	void PDDLEsterelPlanParser::preparePlan() {

		int curr, next;
		std::string line;
		std::istringstream planfile(planner_output);

		size_t planFreeActionID = 0;

		while (std::getline(planfile, line)) {

			if (line.length()<2)
				break;

			// check to see if the line looks like a planned action
			if (line.find("[", 0) == std::string::npos
					|| line.find("]", 0) == std::string::npos
					|| line.find("(", 0) == std::string::npos
					|| line.find(")", 0) == std::string::npos
					|| line.find(":", 0) == std::string::npos)
				continue;

			rosplan_dispatch_msgs::ActionDispatch msg;

			// action ID
			msg.action_id = planFreeActionID;
			planFreeActionID++;

			// dispatchTime
			curr=line.find(":");
			double dispatchTime = (double)atof(line.substr(0,curr).c_str());
			msg.dispatch_time = dispatchTime;

			// check for parameters
			curr=line.find("(")+1;
			bool paramsExist = (line.find(" ",curr) < line.find(")",curr));

			if(paramsExist) {

				// name
				next=line.find(" ",curr);
				std::string name = line.substr(curr,next-curr).c_str();
				msg.name = name;

				// parameters
				std::vector<std::string> params;
				curr=next+1;
				next=line.find(")",curr);
				int at = curr;
				while(at < next) {
					int cc = line.find(" ",curr);
					int cc1 = line.find(")",curr);
					curr = cc<cc1?cc:cc1;
					std::string param = line.substr(at,curr-at);
					params.push_back(param);
					++curr;
					at = curr;
				}
				processPDDLParameters(msg, params);


			} else {

				// name
				next=line.find(")",curr);
				std::string name = line.substr(curr,next-curr).c_str();
				msg.name = name;

			}

			// duration
			curr=line.find("[",curr)+1;
			next=line.find("]",curr);
			msg.duration = (double)atof(line.substr(curr,next-curr).c_str());

			// create action msg
			action_list.push_back(msg);


			// create action start node
			rosplan_dispatch_msgs::EsterelPlanNode node_start;
			node_start.action = msg;
			std::stringstream ss;
			ss << msg.name << "_start";
			node_start.name = ss.str();
			last_plan.nodes.push_back(node_start);
		}

		// find and create causal edges
		createGraph();
	}

	/**
	 * processes the parameters of a single PDDL action into an ActionDispatch message
	 */
	void PDDLEsterelPlanParser::processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params) {

		ros::ServiceClient client = node_handle->serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/kcl_rosplan/get_domain_operator_details");
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = msg.name;
		if(!client.call(srv)) {
			ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s", ros::this_node::getName().c_str(), msg.name.c_str());
		} else {
			// ground and save action
			action_details[msg.action_id] = srv.response.op;

			std::vector<diagnostic_msgs::KeyValue> opParams = srv.response.op.formula.typed_parameters;
			for(size_t i=0; i<opParams.size(); i++) {

				diagnostic_msgs::KeyValue pair;
				pair.key = opParams[i].key;
				pair.value = params[i];
				msg.parameters.push_back(pair);

				std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;

				// rosplan_knowledge_msgs/DomainFormula[] at_start_add_effects
				cit = action_details[msg.action_id].at_start_add_effects.begin();
				for(; cit!=action_details[msg.action_id].at_start_add_effects.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}

				// rosplan_knowledge_msgs/DomainFormula[] at_start_del_effects
				cit = action_details[msg.action_id].at_start_del_effects.begin();
				for(; cit!=action_details[msg.action_id].at_start_del_effects.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}

				// rosplan_knowledge_msgs/DomainFormula[] at_end_add_effects
				cit = action_details[msg.action_id].at_end_add_effects.begin();
				for(; cit!=action_details[msg.action_id].at_end_add_effects.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}

				// rosplan_knowledge_msgs/DomainFormula[] at_end_del_effects
				cit = action_details[msg.action_id].at_end_del_effects.begin();
				for(; cit!=action_details[msg.action_id].at_end_del_effects.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}

				// rosplan_knowledge_msgs/DomainFormula[] at_start_simple_condition
				cit = action_details[msg.action_id].at_start_simple_condition.begin();
				for(; cit!=action_details[msg.action_id].at_start_simple_condition.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}

				// rosplan_knowledge_msgs/DomainFormula[] over_all_simple_condition
				cit = action_details[msg.action_id].over_all_simple_condition.begin();
				for(; cit!=action_details[msg.action_id].over_all_simple_condition.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}

				// rosplan_knowledge_msgs/DomainFormula[] at_end_simple_condition
				cit = action_details[msg.action_id].at_end_simple_condition.begin();
				for(; cit!=action_details[msg.action_id].at_end_simple_condition.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}

				// rosplan_knowledge_msgs/DomainFormula[] at_start_neg_condition
				cit = action_details[msg.action_id].at_start_neg_condition.begin();
				for(; cit!=action_details[msg.action_id].at_start_neg_condition.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}

				// rosplan_knowledge_msgs/DomainFormula[] over_all_neg_condition
				cit = action_details[msg.action_id].over_all_neg_condition.begin();
				for(; cit!=action_details[msg.action_id].over_all_neg_condition.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}

				// rosplan_knowledge_msgs/DomainFormula[] at_end_neg_condition
				cit = action_details[msg.action_id].at_end_neg_condition.begin();
				for(; cit!=action_details[msg.action_id].at_end_neg_condition.end(); cit++) {
					for(int j=0;j<cit->typed_parameters.size();j++) {
						if(opParams[i].key == cit->typed_parameters[j].key) {
							cit->typed_parameters[j].value = params[i];
						}
					}
				}
			}
		}
	}

	/*--------------*/
	/* Create graph */
	/*--------------*/
	
	void PDDLEsterelPlanParser::createGraph() {

		std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ait = last_plan.nodes.begin();
		for(; ait!=last_plan.nodes.end(); ait++) {
	
			// iterate through conditions
			rosplan_knowledge_msgs::DomainOperator op = action_details[ait->action.action_id];
			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;

			// rosplan_knowledge_msgs/DomainFormula[] at_start_simple_condition
			cit = op.at_start_simple_condition.begin();
			for(; cit!=op.at_start_simple_condition.end(); cit++) {
				// for this condition create possible causal edges
				createEdge(ait, *cit, false);
			}

			// rosplan_knowledge_msgs/DomainFormula[] over_all_simple_condition
			cit = op.over_all_simple_condition.begin();
			for(; cit!=op.over_all_simple_condition.end(); cit++) {
				// for this condition create possible causal edges
				createEdge(ait, *cit, false);
			}

			// rosplan_knowledge_msgs/DomainFormula[] at_end_simple_condition
			cit = op.at_end_simple_condition.begin();
			for(; cit!=op.at_end_simple_condition.end(); cit++) {
				// for this condition create possible causal edges
				createEdge(ait, *cit, false);
			}

			// rosplan_knowledge_msgs/DomainFormula[] at_start_neg_condition
			cit = op.at_start_neg_condition.begin();
			for(; cit!=op.at_start_neg_condition.end(); cit++) {
				// for this condition create possible causal edges
				createEdge(ait, *cit, true);
			}

			// rosplan_knowledge_msgs/DomainFormula[] over_all_neg_condition
			cit = op.over_all_neg_condition.begin();
			for(; cit!=op.over_all_neg_condition.end(); cit++) {
				// for this condition create possible causal edges
				createEdge(ait, *cit, true);
			}

			// rosplan_knowledge_msgs/DomainFormula[] at_end_neg_condition
			cit = op.at_end_neg_condition.begin();
			for(; cit!=op.at_end_neg_condition.end(); cit++) {
				// for this condition create possible causal edges
				createEdge(ait, *cit, true)

			// this line is getting deleted;
			}
		}
	}

	/**
	 * finds the previous supporting action and creates a new edge.
	 * @returns True if an edge is created.
	 */
	bool PDDLEsterelPlanParser::createEdge(std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ait, rosplan_knowledge_msgs::DomainFormula& condition, bool negative_condition) {
		// for this condition check previous actions
		std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_reverse_iterator pait(ait);
		for(; pait!=last_plan.nodes.rend(); pait++) {

			// supports
			if(satisfiesPrecondition(condition, *pait, negative_condition)) {

				// if action satisfies condition then create and add edge
				rosplan_dispatch_msgs::EsterelPlanEdge newEdge;
				newEdge.edge_id = last_plan.edges.size();
				std::stringstream ss;
				ss << "edge" << "_" << newEdge.edge_id << " " << condition.name;
				newEdge.edge_name = ss.str();
				newEdge.signal_type = 0;
				newEdge.source_ids.push_back(pait->action.action_id);
				newEdge.sink_ids.push_back(ait->action.action_id);

				last_plan.edges.push_back(newEdge);

				last_plan.nodes[pait->action.action_id].edges_out.push_back(newEdge.edge_id);
				last_plan.nodes[ait->action.action_id].edges_in.push_back(newEdge.edge_id);


				return true;
			}
		}
		return false;
	}

	/**
	 * @returns True if the node satisfies the (possibly negative) condition
	 */
	bool PDDLEsterelPlanParser::satisfiesPrecondition(rosplan_knowledge_msgs::DomainFormula& condition, const rosplan_dispatch_msgs::EsterelPlanNode& node, bool negative_condition) {
		
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;

		if(!negative_condition) {

			// rosplan_knowledge_msgs/DomainFormula[] at_start_add_effects
			cit = action_details[node.action.action_id].at_start_add_effects.begin();
			for(; cit!=action_details[node.action.action_id].at_start_add_effects.end(); cit++) {
				if(domainFormulaMatches(condition, *cit)) return true;
			}

			// rosplan_knowledge_msgs/DomainFormula[] at_end_add_effects
			cit = action_details[node.action.action_id].at_end_add_effects.begin();
			for(; cit!=action_details[node.action.action_id].at_end_add_effects.end(); cit++) {
				if(domainFormulaMatches(condition, *cit)) return true;
			}

		} else {

			// rosplan_knowledge_msgs/DomainFormula[] at_start_del_effects
			cit = action_details[node.action.action_id].at_start_del_effects.begin();
			for(; cit!=action_details[node.action.action_id].at_start_del_effects.end(); cit++) {
				if(domainFormulaMatches(condition, *cit)) return true;
			}

			// rosplan_knowledge_msgs/DomainFormula[] at_end_del_effects
			cit = action_details[node.action.action_id].at_end_del_effects.begin();
			for(; cit!=action_details[node.action.action_id].at_end_del_effects.end(); cit++) {
				if(domainFormulaMatches(condition, *cit)) return true;
			}
		}
	}

} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc,argv,"rosplan_plan_parser");
		ros::NodeHandle nh("~");

		KCL_rosplan::PDDLEsterelPlanParser pp(nh);
	
		// subscribe to planner output
		std::string planTopic = "planner_output";
		nh.getParam("planner_topic", planTopic);
		ros::Subscriber plan_sub = nh.subscribe(planTopic, 1, &KCL_rosplan::PlanParser::plannerCallback, dynamic_cast<KCL_rosplan::PlanParser*>(&pp));
	
		// start the plan parsing services
		ros::ServiceServer service1 = nh.advertiseService("parse_plan", &KCL_rosplan::PlanParser::parsePlan, dynamic_cast<KCL_rosplan::PlanParser*>(&pp));
		ros::ServiceServer service2 = nh.advertiseService("parse_plan_from_file", &KCL_rosplan::PlanParser::parsePlanFromFile, dynamic_cast<KCL_rosplan::PlanParser*>(&pp));

		ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
		ros::spin();

		return 0;
	}
