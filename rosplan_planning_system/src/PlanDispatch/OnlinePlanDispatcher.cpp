//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#include <rosplan_dispatch_msgs/GetPlanningParams.h>
#include "rosplan_planning_system/PlanDispatch/OnlinePlanDispatcher.h"
namespace KCL_rosplan {


    OnlinePlanDispatcher::OnlinePlanDispatcher(ros::NodeHandle &nh) : PlanDispatcher(nh) {
        node_handle = &nh;

        // get port
        server_port_ = 3234;
        node_handle->getParam("ippc_server_port", server_port_);
        node_handle->param("compute_rewards", compute_rewards_, false);

        // knowledge base services
        std::stringstream ss;
        ss << "/" << kb_ << "/state/propositions";
        queryPropositionsClient = node_handle->serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());
        ss.str("");

        ss << "/" << kb_ << "/state/rddl_parameters";
        get_rddl_params = node_handle->serviceClient<rosplan_knowledge_msgs::GetRDDLParams>(ss.str());
        ss.str("");

        ss << "/" << kb_ << "/state/get_immediate_reward";
        get_immediate_reward = node_handle->serviceClient<rosplan_knowledge_msgs::GetRDDLImmediateReward>(ss.str());
        ss.str("");

        get_planner_params =  node_handle->serviceClient<rosplan_dispatch_msgs::GetPlanningParams>("/rosplan_planner_interface/get_planning_params");

        // subscribe to planner output
        std::string planTopic = "complete_plan";
        nh.getParam("plan_topic", planTopic);
        plan_publisher = node_handle->advertise<rosplan_dispatch_msgs::CompletePlan>(planTopic, 1, true);

        reset();
    }

    OnlinePlanDispatcher::~OnlinePlanDispatcher() {}

    void OnlinePlanDispatcher::reset() {
        PlanDispatcher::reset();
        current_action = 0;
    }

    /*--------------------*/
    /* Dispatch interface */
    /*--------------------*/

    /**
     * plan dispatch service method (1)
     * dispatches plan as a service
     * @returns True iff every action was dispatched and returned success.
     */
    bool OnlinePlanDispatcher::dispatchPlanService(rosplan_dispatch_msgs::DispatchService::Request &req, rosplan_dispatch_msgs::DispatchService::Response &res) {
        mission_start_time = ros::WallTime::now().toSec();
        PlanDispatcher::dispatchPlanService(req, res);
    }

    /**
     * plan dispatch action method (2)
     * dispatches plan as a service
     * @returns True iff every action was dispatched and returned success.
     */
    void OnlinePlanDispatcher::dispatchPlanActionlib() {
        mission_start_time = ros::WallTime::now().toSec();
        PlanDispatcher::dispatchPlanActionlib();
    }

    /*-----------------*/
    /* action dispatch */
    /*-----------------*/

    /*
     * Loop through and publish planned actions
     */

    bool KCL_rosplan::OnlinePlanDispatcher::dispatchPlan(double missionStartTime, double planStartTime) {
        ROS_INFO("KCL: (%s) Dispatching online plan", ros::this_node::getName().c_str());

        // Get horizon
        rosplan_knowledge_msgs::GetRDDLParams srvparams;
        if (not get_rddl_params.call(srvparams)) {
            ROS_ERROR("KCL: (%s) Failed to call service to get RDDL parameters! Are you using a RDDL KnowledgeBase?", ros::this_node::getName().c_str());
            ros::shutdown();
            return false;
        }

        rosplan_dispatch_msgs::GetPlanningParams p;
        if (not get_planner_params.call(p)) {
            ROS_ERROR("KCL: (%s) Failed to call service to get planner parameters! Is the OnlinePlanningInterface running?", ros::this_node::getName().c_str());
            ros::shutdown();
            return false;
        }

        if (not p.response.planner_ready) {
            ROS_WARN("KCL: (%s) Planner is not ready! Did you call the plan_server to start the online planner thread?", ros::this_node::getName().c_str());
            return false;
        }

        // Start round
        ROS_INFO("KCL: (%s) Starting IPPC server on port %d and waiting for connections!", ros::this_node::getName().c_str(), server_port_);
        XMLServer_t ippcserver;
        std::string client = ippcserver.start_session(server_port_, p.response.domain_path, p.response.problem_path);
        ROS_INFO("KCL: (%s) Planner %s connected! Starting planning round", ros::this_node::getName().c_str(), client.c_str());
        ippcserver.start_round();
        float planning_result;
        ros::Rate loop_rate(10);
        ros::Duration total_planning_time(0);
        std::string action; // Action name
        bool first_action = true; // To set reward 0 at the first planning round
        std::regex commas (",(\\S)"); // Matches commas without a space

        // Loop over the horizon
        for (int horizon = 0; horizon < srvparams.response.horizon; ++horizon) {
            while (ros::ok() && dispatch_paused) {
                ros::spinOnce();
                loop_rate.sleep();
            }

            // cancel plan
            if (plan_cancelled) {
                break;
            }

            // Query current state
            rosplan_knowledge_msgs::GetAttributeService srv;
            if (not queryPropositionsClient.call(srv)) ROS_ERROR("KCL: (%s) Failed to call service to get current state.", ros::this_node::getName().c_str());

            try {
                ros::Time start = ros::Time::now();
                double reward = 0;
                if (compute_rewards_ and not first_action) {
                    rosplan_knowledge_msgs::GetRDDLImmediateReward rwd_srv;
                    if (action.empty()) action = "noop";
                    std::istringstream ss(action);
                    std::string a;
                    while (std::getline(ss, a, ';')) {
                        std_msgs::String msg;
                        std::regex_replace (std::back_inserter(msg.data), a.begin(), a.end(), commas, ", $1"); // Adds a space after the commas for the param list. Needed for the reward computation
                        // Note: \S matches the first letter of the parameter, so we add it back again
                        rwd_srv.request.action.push_back(msg);
                    }
                    if (not get_immediate_reward.call(rwd_srv)) ROS_ERROR("KCL: (%s) Failed to call service to get immediate reward.", ros::this_node::getName().c_str());
                    else reward = rwd_srv.response.reward;
                }
                else first_action = false;
                action = ippcserver.get_action(srv.response.attributes, planning_result, reward);
                total_planning_time += ros::Time::now()-start;
            }
            catch (std::runtime_error e) {
                ROS_ERROR("KCL: (%s) %s", ros::this_node::getName().c_str(), e.what());
                break;
            }

            if (action.empty()) continue; // It is a noop
            ROS_INFO("KCL: (%s) Received new action from planner: %s", ros::this_node::getName().c_str(), action.c_str());

            std::vector<rosplan_dispatch_msgs::ActionDispatch> current_action_msg = toActionMsg(action, current_action);
            for (auto it = current_action_msg.begin(); it != current_action_msg.end(); ++it) {
                dispatchOneAction(*it, missionStartTime, planStartTime);
                current_plan.plan.push_back(*it);
                plan_publisher.publish(current_plan);

                // get ready for next action
                ++current_action;
                action_received[current_action] = false;
                action_completed[current_action] = false;
            }

        }
        ippcserver.end_round();
        ros::Duration(0.5).sleep();
        ippcserver.end_session();

        ROS_INFO("KCL: (%s) Dispatch complete.", ros::this_node::getName().c_str());
        ROS_INFO("KCL: (%s) Total planning time: %f seconds", ros::this_node::getName().c_str(), total_planning_time.toSec());
        return true;
    }


    /*------------------*/
    /* general feedback */
    /*------------------*/

    /**
     * listen to and process actionFeedback topic.
     */
    void OnlinePlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

        // create error if the action is unrecognised
        ROS_INFO("KCL: (%s) Feedback received [%i, %d]", ros::this_node::getName().c_str(), msg->action_id, msg->status);
        if(current_action != (unsigned int)msg->action_id)
            ROS_ERROR("KCL: (%s) Unexpected action ID: %d; current action: %d", ros::this_node::getName().c_str(), msg->action_id, current_action);

        // action enabled
        if(!action_received[msg->action_id] && msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_ENABLED)
            action_received[msg->action_id] = true;

        // action completed (successfuly)
        else if(!action_completed[msg->action_id] && msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_SUCCEEDED_TO_GOAL_STATE)
            action_completed[msg->action_id] = true;

        // action completed (failed)
        else if(!action_completed[msg->action_id] && msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_FAILED) {
            replan_requested = true;
            action_completed[msg->action_id] = true;
        }

        // action completed (failed)
        else if(!action_completed[msg->action_id] && msg->status == rosplan_dispatch_msgs::ActionFeedback::ACTION_PRECONDITION_FALSE) {
            replan_requested = true;
            action_completed[msg->action_id] = true;
        }
    }

    void OnlinePlanDispatcher::dispatchOneAction(rosplan_dispatch_msgs::ActionDispatch current_action_msg,
                                                 double missionStartTime, double planStartTime) {
        // check action preconditions
        if(!checkPreconditions(current_action_msg)) {
            ROS_WARN("KCL: (%s) Preconditions not achieved [%i, %s]", ros::this_node::getName().c_str(), current_action_msg.action_id, current_action_msg.name.c_str());

            // publish feedback (precondition false)
            rosplan_dispatch_msgs::ActionFeedback fb;
            fb.action_id = current_action_msg.action_id;
            fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_PRECONDITION_FALSE;
            publishFeedback(fb);

            replan_requested = true;

        } else {

            // dispatch action
            ROS_INFO("KCL: (%s) Dispatching action [%i, %s, %f, %f]",
                     ros::this_node::getName().c_str(),
                     current_action_msg.action_id,
                     current_action_msg.name.c_str(),
                     (current_action_msg.dispatch_time + planStartTime - missionStartTime),
                     current_action_msg.duration);

            action_dispatch_publisher.publish(current_action_msg);
            // publish feedback (action dispatched)
            rosplan_dispatch_msgs::ActionFeedback fb;
            fb.action_id = current_action_msg.action_id;
            fb.status = rosplan_dispatch_msgs::ActionFeedback::ACTION_DISPATCHED_TO_GOAL_STATE;
            publishFeedback(fb);

            double late_print = (ros::WallTime::now().toSec() - (current_action_msg.dispatch_time + planStartTime));
            if (late_print > 0.1) {
                ROS_INFO("KCL: (%s) Action [%i] is %f second(s) late", ros::this_node::getName().c_str(),
                         current_action_msg.action_id, late_print);
            }
        }
            // wait for action to complete
            ros::Rate loop_rate(10);
            while (ros::ok() && !action_completed[current_action]) {
                ros::spinOnce();
                loop_rate.sleep();
            }

    }

    std::vector<rosplan_dispatch_msgs::ActionDispatch> OnlinePlanDispatcher::toActionMsg(std::string resp_actions, int action_id) {
        std::vector<rosplan_dispatch_msgs::ActionDispatch>  ret;
        std::regex action_name_params_rgx("(.+)\\((.*)\\)|,?\\s?(.+)");
        std::smatch match;

        std::istringstream action_list(resp_actions);
        std::string action_str;
        while (std::getline(action_list, action_str, ';')) { // Get each action separately
            rosplan_dispatch_msgs::ActionDispatch action_msg;
            action_msg.action_id = action_id;
            if (std::regex_search(action_str, match, action_name_params_rgx)) { // Separate action name and parameters
                std::string action_name = (match[1].str().empty()) ? match[3].str() : match[1].str();
                action_msg.name = action_name; // Action name

                // Get action details to instantiate the parameters
                rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
                srv.request.name = action_name;
                if (!queryDomainClient.call(srv)) {
                    ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s",
                              ros::this_node::getName().c_str(), srv.response.op.formula.name.c_str());
                    return ret;
                }

                // Process parameters
                action_msg.parameters = srv.response.op.formula.typed_parameters;
                std::istringstream p(match[2].str()); // parameters
                std::string param;
                size_t param_idx = 0;
                while (getline(p, param, ',')) {
                    action_msg.parameters[param_idx].value = param;
                    ++param_idx;
                }
            }
            action_msg.duration = 0.001;
            action_msg.dispatch_time = ros::WallTime::now().toSec();
            ret.push_back(action_msg);
            ++action_id;
        }

        return ret;
    }


} // close namespace

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv) {

    ros::init(argc,argv,"rosplan_plan_dispatcher");
    ros::NodeHandle nh("~");

    KCL_rosplan::OnlinePlanDispatcher opd(nh);

    std::string feedbackTopic = "action_feedback";
    nh.getParam("action_feedback_topic", feedbackTopic);
    ros::Subscriber feedback_sub = nh.subscribe(feedbackTopic, 1, &KCL_rosplan::OnlinePlanDispatcher::feedbackCallback, &opd);

    ROS_INFO("KCL: (%s) Ready to receive", ros::this_node::getName().c_str());
    ros::spin();

    return 0;
}

