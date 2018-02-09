#include "rosplan_planning_system/PlanParsing/PDDLEsterelPlanParser.h"

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
			node_start.node_type = rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START;
			node_start.node_id = last_plan.nodes.size();
			node_start.action = msg;
			std::stringstream ss;
			ss << msg.name << "_start";
			node_start.name = ss.str();
			last_plan.nodes.push_back(node_start);

			// create action end node
			rosplan_dispatch_msgs::EsterelPlanNode node_end;
			node_end.node_type = rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END;
			node_end.node_id = last_plan.nodes.size();
			node_end.action = msg;
			ss.str("");
			ss << msg.name << "_end";
			node_end.name = ss.str();
			last_plan.nodes.push_back(node_end);
		}

		// find and create causal edges
		createGraph();
	}

	/**
	 * processes the parameters of a single PDDL action into an ActionDispatch message
	 */
	void PDDLEsterelPlanParser::processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params) {

		// fetch operator details
		ros::ServiceClient client = node_handle->serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/kcl_rosplan/get_domain_operator_details");
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = msg.name;
		if(!client.call(srv)) {
			ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s", ros::this_node::getName().c_str(), msg.name.c_str());
		} else {

			action_details[msg.action_id] = srv.response.op;

			// save parameters in action message
			std::vector<diagnostic_msgs::KeyValue> opParams = srv.response.op.formula.typed_parameters;
			for(size_t i=0; i<opParams.size(); i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = opParams[i].key;
				pair.value = params[i];
				msg.parameters.push_back(pair);
			}

			// ground and save action details
			groundFormula(action_details[msg.action_id].at_start_add_effects, opParams, params);
			groundFormula(action_details[msg.action_id].at_start_del_effects, opParams, params);

			groundFormula(action_details[msg.action_id].at_end_add_effects, opParams, params);
			groundFormula(action_details[msg.action_id].at_end_del_effects, opParams, params);

			groundFormula(action_details[msg.action_id].at_start_simple_condition, opParams, params);
			groundFormula(action_details[msg.action_id].at_end_simple_condition, opParams, params);
			groundFormula(action_details[msg.action_id].over_all_simple_condition, opParams, params);

			groundFormula(action_details[msg.action_id].at_start_neg_condition, opParams, params);
			groundFormula(action_details[msg.action_id].at_end_neg_condition, opParams, params);
			groundFormula(action_details[msg.action_id].over_all_neg_condition, opParams, params);
		}
	}

	/*--------------*/
	/* Create graph */
	/*--------------*/
	
	void PDDLEsterelPlanParser::createGraph() {

		// map of absolute plan time to node ID
		std::map<double,int> nodes;

		// construct ordered list of nodes
		std::vector<rosplan_dispatch_msgs::EsterelPlanNode>::const_iterator ait = last_plan.nodes.begin();
		for(; ait!=last_plan.nodes.end(); ait++) {

			// get node time
			double time = ait->action.dispatch_time;
			if(ait->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END)
				time = time + ait->action.duration;

			// find a unique time for the node
			while(nodes.find(time)!=nodes.end())
				time = time + 0.1;

			nodes.insert(std::pair<double,int>(time,ait->node_id));
		}		

		// get the next node
		std::map<double,int>::iterator nit = nodes.begin();
		for(; nit!=nodes.end(); nit++) {
			
			rosplan_dispatch_msgs::EsterelPlanNode *node = &last_plan.nodes[nit->second];

			// if action end, insert edge from action start
			if(node->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {
				makeEdge(node->node_id-1, node->node_id);
			}

			// for each precondition add possible causal support edge with latest preceding node
			rosplan_knowledge_msgs::DomainOperator op = action_details[node->action.action_id];
			std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit;

			if(node->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {

				for(cit = op.at_start_simple_condition.begin(); cit!=op.at_start_simple_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, false);
				for(cit = op.over_all_simple_condition.begin(); cit!=op.over_all_simple_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, false);
				for(cit = op.at_start_neg_condition.begin(); cit!=op.at_start_neg_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, true);
				for(cit = op.over_all_neg_condition.begin(); cit!=op.over_all_neg_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, true);

			} else if(node->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {

				for(cit = op.at_end_simple_condition.begin(); cit!=op.at_end_simple_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, false);
				for(cit = op.at_end_neg_condition.begin(); cit!=op.at_end_neg_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, true);
			}

			// interference edges
			// if current action has precondition order after preceding node(s) with negating effect
			if(node->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_START) {

				for(cit = op.at_start_simple_condition.begin(); cit!=op.at_start_simple_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, false);
				for(cit = op.over_all_simple_condition.begin(); cit!=op.over_all_simple_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, false);
				for(cit = op.at_start_neg_condition.begin(); cit!=op.at_start_neg_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, true);
				for(cit = op.over_all_neg_condition.begin(); cit!=op.over_all_neg_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, true);

			} else if(node->node_type == rosplan_dispatch_msgs::EsterelPlanNode::ACTION_END) {

				for(cit = op.at_end_simple_condition.begin(); cit!=op.at_end_simple_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, false);
				for(cit = op.at_end_neg_condition.begin(); cit!=op.at_end_neg_condition.end(); cit++)
					addConditionEdge(nodes, nit, *cit, true);
			}
		}
	}

	/**
	 * finds the previous supporting action and creates a new edge.
	 * @returns True if an edge is created.
	 */
	bool PDDLEsterelPlanParser::addConditionEdge(std::map<double,int> &node_map, std::map<double,int>::iterator &current_node, rosplan_knowledge_msgs::DomainFormula &condition, bool negative_condition) {

		// for this condition check previous actions
		std::map<double,int>::const_reverse_iterator rit(current_node);
		for(; rit!=node_map.rend(); rit++) {
			// check support
			if(satisfiesPrecondition(condition, last_plan.nodes[rit->second], negative_condition)) {
				makeEdge(rit->second, current_node->second);
				return true;
			}
		}
		return false;
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
