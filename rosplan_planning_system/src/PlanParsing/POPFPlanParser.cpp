#include "rosplan_planning_system/PlanParsing/POPFPlanParser.h"

namespace KCL_rosplan {

	POPFPlanParser::POPFPlanParser(ros::NodeHandle& nh)
	{
		node_handle = &nh;

		// publishing parsed plan
		std::string planTopic = "complete_plan";
		node_handle->getParam("plan_topic", planTopic);
		plan_publisher = node_handle->advertise<rosplan_dispatch_msgs::CompletePlan>(planTopic, 1, true);
	}

	POPFPlanParser::~POPFPlanParser()
	{

	}

	void POPFPlanParser::reset() {
		action_list.clear();
	}

	void POPFPlanParser::publishPlan() {
		rosplan_dispatch_msgs::CompletePlan msg;
		msg.plan = action_list;
		plan_publisher.publish(msg);
	}

	/*----------------------*/
	/* Post processing plan */
	/*----------------------*/

	/**
	 * parses the output of popf, generating a list of action messages.
	 */
	void POPFPlanParser::preparePlan() {
		
		std::vector<rosplan_dispatch_msgs::ActionDispatch> potentialPlan;

		double planDuration;
		double expectedPlanDuration = 0;

		int curr, next; 
		std::string line;
		std::istringstream planfile(planner_output);
		while (std::getline(planfile, line)) {

			if (line.substr(0,6).compare("; Plan") == 0) {
				expectedPlanDuration = atof(line.substr(25).c_str());
			} else if (line.substr(0,6).compare("; Time")!=0) {
				//consume useless lines
			} else {

				potentialPlan.clear();
				size_t planFreeActionID = 0;
				planDuration = 0;

				while (std::getline(planfile, line)) {

					if (line.length()<2)
						break;

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

					potentialPlan.push_back(msg);

					// update plan duration
					curr=line.find(":");
					planDuration = msg.duration + atof(line.substr(0,curr).c_str());
				}

				if(planDuration - expectedPlanDuration < 0.01)  {

					// save better optimised plan
					for(size_t i=0;i<potentialPlan.size();i++) {
						action_list.push_back(potentialPlan[i]);
					}

					total_plan_duration = planDuration;

				} else {
					ROS_INFO("KCL: (%s) Duration: %f, expected %f; plan discarded", ros::this_node::getName().c_str(), planDuration, expectedPlanDuration);
				}
			}
		}
	}

	/**
	 * processes the parameters of a single PDDL action into an ActionDispatch message
	 */
	void POPFPlanParser::processPDDLParameters(rosplan_dispatch_msgs::ActionDispatch &msg, std::vector<std::string> &params) {

		ros::ServiceClient client = node_handle->serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>("/kcl_rosplan/get_domain_operator_details");
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = msg.name;
		if(!client.call(srv)) {
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

		KCL_rosplan::POPFPlanParser pp(nh);
	
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
