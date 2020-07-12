#include "rosplan_planning_system/PlanParsing/PDDLSimplePlanParser.h"

namespace KCL_rosplan {

	PDDLSimplePlanParser::PDDLSimplePlanParser(ros::NodeHandle& nh)
	{
		node_handle = &nh;

		// fetching problem info for TILs
		std::string kb = "knowledge_base";
		node_handle->getParam("knowledge_base", kb);

		std::stringstream ss;
		ss << "/" << kb << "/domain/operator_details";
		get_operator_details_client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str().c_str());
		ss.str("");

		// publishing parsed plan
		std::string planTopic = "complete_plan";
		node_handle->getParam("plan_topic", planTopic);
		plan_publisher = node_handle->advertise<rosplan_dispatch_msgs::CompletePlan>(planTopic, 1, true);
	}

	PDDLSimplePlanParser::~PDDLSimplePlanParser()
	{
	}

	void PDDLSimplePlanParser::reset() {
        
		action_list.clear();
	}

	void PDDLSimplePlanParser::publishPlan() {
        
		ROS_INFO("KCL: (%s) Plan published.", ros::this_node::getName().c_str());    
		ROS_INFO("KCL: (%s) Is plan empty?: %d", ros::this_node::getName().c_str(), action_list.size() == 0);
 		ROS_DEBUG("KCL: (%s) Num actions: %d", ros::this_node::getName().c_str(), action_list.size());
        
		rosplan_dispatch_msgs::CompletePlan msg;
		msg.plan = action_list;
		plan_publisher.publish(msg);
	}

	/*----------------------*/
	/* Post processing plan */
	/*----------------------*/

	/**
	 * parses standard PDDL output, generating a list of action messages.
	 */
	void PDDLSimplePlanParser::preparePlan() {

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

			action_list.push_back(msg);
		}
	}

	/**
	 * processes the parameters of a single PDDL action into an ActionDispatch message
	 */
	void PDDLSimplePlanParser::processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params) {

		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = msg.name;
		if(!get_operator_details_client.call(srv)) {
			ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s", ros::this_node::getName().c_str(), msg.name.c_str());
		} else {
			std::vector<diagnostic_msgs::KeyValue> opParams = srv.response.op.formula.typed_parameters;
			for(size_t i=0; i<opParams.size(); i++) {
				diagnostic_msgs::KeyValue pair;
				pair.key = opParams[i].key;
				pair.value = params[i];
				msg.parameters.push_back(pair);
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

    KCL_rosplan::PDDLSimplePlanParser pp(nh);

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
