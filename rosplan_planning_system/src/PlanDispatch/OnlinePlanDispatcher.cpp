//
// Created by Gerard Canal <gcanal@iri.upc.edu> on 11/10/18.
//

#include "rosplan_planning_system/PlanDispatch/OnlinePlanDispatcher.h"
namespace KCL_rosplan {


    OnlinePlanDispatcher::OnlinePlanDispatcher(ros::NodeHandle &nh) : PlanDispatcher(nh) {
        node_handle = &nh;

        // get port
        server_port_ = 3234;
        node_handle->getParam("ippc_server_port", server_port_);

        // knowledge base services
        std::string kb = "knowledge_base";
        node_handle->getParam("knowledge_base", kb);
        std::stringstream ss;
        ss << "/" << kb << "/query_state";
        queryKnowledgeClient = node_handle->serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>(ss.str());
        ss.str("");
        ss << "/" << kb << "/state/propositions";
        queryPropositionsClient = node_handle->serviceClient<rosplan_knowledge_msgs::GetAttributeService>(ss.str());
        ss.str("");
        ss << "/" << kb << "/domain/operator_details";
        queryDomainClient = node_handle->serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str());
        ss.str("");

        ss << "/" << kb << "/state/rddl_parameters";
        get_rddl_params = node_handle->serviceClient<rosplan_knowledge_msgs::GetRDDLParams>(ss.str());
        ss.str("");

        action_dispatch_topic = "action_dispatch";
        action_feedback_topic = "action_feedback";
        nh.getParam("action_dispatch_topic", action_dispatch_topic);
        nh.getParam("action_feedback_topic", action_feedback_topic);
        action_dispatch_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionDispatch>(action_dispatch_topic, 1, true);
        action_feedback_publisher = node_handle->advertise<rosplan_dispatch_msgs::ActionFeedback>(action_feedback_topic, 1, true);

        // subscribe to planner output
        std::string planTopic = "complete_plan";
        nh.getParam("plan_topic", planTopic);
        plan_publisher = node_handle->advertise<rosplan_dispatch_msgs::CompletePlan>(planTopic, 1, true);

        reset();
    }

    OnlinePlanDispatcher::~OnlinePlanDispatcher() {}

    void OnlinePlanDispatcher::reset() {
        replan_requested = false;
        dispatch_paused = false;
        plan_cancelled = false;
        action_received.clear();
        action_completed.clear();
        plan_received = false;
        current_action = 0;
        current_plan.plan.clear();
    }

    /*--------------------*/
    /* Dispatch interface */
    /*--------------------*/

    /**
     * plan dispatch service method (1)
     * dispatches plan as a service
     * @returns True iff every action was dispatched and returned success.
     */
    bool OnlinePlanDispatcher::dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        if (dispatching) return false;
        dispatching = true;
        mission_start_time = ros::WallTime::now().toSec();
        bool success = dispatchPlan(mission_start_time,ros::WallTime::now().toSec());
        dispatching = false;
        reset();
        return success;
    }

    /**
     * plan dispatch action method (2)
     * dispatches plan as a service
     * @returns True iff every action was dispatched and returned success.
     */
    void OnlinePlanDispatcher::dispatchPlanActionlib() {
        if (as_.isActive() or dispatching) {
            ROS_WARN("KCL: (%s) Got a new dispatch request but a plan is already being dispatched! It will be ignored", ros::this_node::getName().c_str());
        }
        else {
            as_.acceptNewGoal();
            dispatching = true;
            mission_start_time = ros::WallTime::now().toSec();
            bool success = dispatchPlan(mission_start_time, ros::WallTime::now().toSec());
            dispatching = false;
            reset();
            rosplan_dispatch_msgs::NonBlockingDispatchResult res;
            res.success = success;
            as_.setSucceeded(res);
        }
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

        // Start round
        ROS_INFO("KCL: (%s) Starting IPPC server on port %d and waiting for connections!", ros::this_node::getName().c_str(), server_port_);
        XMLServer_t ippcserver;
        ippcserver.start_session(server_port_);
        ROS_INFO("KCL: (%s) Starting planning round", ros::this_node::getName().c_str());
        ippcserver.start_round();
        float planning_result;
        ros::Rate loop_rate(10);

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

            // Get next action
            std::string action;
            try {
                action = ippcserver.get_action(srv.response.attributes, planning_result);
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

        // TODO add infinite dispatch?
        ROS_INFO("KCL: (%s) Dispatch complete.", ros::this_node::getName().c_str());
        return true;
    }

    /**
     *	Returns true of the actions preconditions are true in the current state. Calls the Knowledge Base.
     */
    bool OnlinePlanDispatcher::checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg) {

        // get domain opertor details
        rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
        srv.request.name = msg.name;
        if (!queryDomainClient.call(srv)) {
            ROS_ERROR("KCL: (%s) could not call Knowledge Base for operator details, %s",
                      ros::this_node::getName().c_str(), msg.name.c_str());
            return false;
        }

        // setup service call
        rosplan_knowledge_msgs::DomainOperator op = srv.response.op;
        rosplan_knowledge_msgs::KnowledgeQueryService querySrv;

        // iterate through conditions
        std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator cit = op.at_start_simple_condition.begin();
        for (; cit != op.at_start_simple_condition.end(); cit++) {

            // create condition
            rosplan_knowledge_msgs::KnowledgeItem condition;
            condition.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
            condition.attribute_name = cit->name;

            // populate parameters
            for (int i = 0; i < cit->typed_parameters.size(); i++) {

                // set parameter label to predicate label
                diagnostic_msgs::KeyValue param;
                param.key = cit->typed_parameters[i].key;

                // search for correct operator parameter value
                for (int j = 0; j < msg.parameters.size() && j < op.formula.typed_parameters.size(); j++) {
                    if (op.formula.typed_parameters[j].key == cit->typed_parameters[i].value) {
                        param.value = msg.parameters[j].value;
                    }
                }
                condition.values.push_back(param);
            }
            querySrv.request.knowledge.push_back(condition);
        }

        // check conditions in knowledge base
        if (queryKnowledgeClient.call(querySrv)) {

            if (!querySrv.response.all_true) {
                std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator kit;
                for (kit = querySrv.response.false_knowledge.begin();
                     kit != querySrv.response.false_knowledge.end(); kit++)
                    ROS_INFO("KCL: (%s) Precondition not achieved: %s", ros::this_node::getName().c_str(),
                             kit->attribute_name.c_str());
            }
            return querySrv.response.all_true;

        } else {
            ROS_ERROR("KCL: (%s) Failed to call service query_state", ros::this_node::getName().c_str());
        }
    }


    /*------------------*/
    /* general feedback */
    /*------------------*/

    /**
     * listen to and process actionFeedback topic.
     */
    void OnlinePlanDispatcher::feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg) {

        // create error if the action is unrecognised
        ROS_INFO("KCL: (%s) Feedback received [%i, %s]", ros::this_node::getName().c_str(), msg->action_id, msg->status.c_str());
        if(current_action != (unsigned int)msg->action_id)
            ROS_ERROR("KCL: (%s) Unexpected action ID: %d; current action: %d", ros::this_node::getName().c_str(), msg->action_id, current_action);

        // action enabled
        if(!action_received[msg->action_id] && (0 == msg->status.compare("action enabled")))
            action_received[msg->action_id] = true;

        // action completed (successfuly)
        if(!action_completed[msg->action_id] && 0 == msg->status.compare("action achieved"))
            action_completed[msg->action_id] = true;

        // action completed (failed)
        if(!action_completed[msg->action_id] && 0 == msg->status.compare("action failed")) {
            replan_requested = true;
            action_completed[msg->action_id] = true;
        }
    }

    void OnlinePlanDispatcher::dispatchOneAction(rosplan_dispatch_msgs::ActionDispatch current_action_msg,
                                                 double missionStartTime, double planStartTime) {
        // check action preconditions
        if(!checkPreconditions(current_action_msg)) {
            ROS_INFO("KCL: (%s) Preconditions not achieved [%i, %s]", ros::this_node::getName().c_str(), current_action_msg.action_id, current_action_msg.name.c_str());

            // publish feedback (precondition false)
            rosplan_dispatch_msgs::ActionFeedback fb;
            fb.action_id = current_action_msg.action_id;
            fb.status = "precondition false";
            publishFeedback(fb);

            replan_requested = true;

        } else {

            // dispatch action
            ROS_INFO("KCL: (%s) Dispatching action [%i, %s, %f, %f]",
                     ros::this_node::getName().c_str(),
                     current_action_msg.action_id,
                     current_action_msg.name.c_str(),
                     (current_action_msg.dispatch_time+planStartTime-missionStartTime),
                     current_action_msg.duration);

            action_dispatch_publisher.publish(current_action_msg);
            // publish feedback (action dispatched)
            rosplan_dispatch_msgs::ActionFeedback fb;
            fb.action_id = current_action_msg.action_id;
            fb.status = "action dispatched";
            publishFeedback(fb);

            double late_print = (ros::WallTime::now().toSec() - (current_action_msg.dispatch_time+planStartTime));
            if(late_print>0.1) {
                ROS_INFO("KCL: (%s) Action [%i] is %f second(s) late", ros::this_node::getName().c_str(), current_action_msg.action_id, late_print);
            }

            // wait for action to complete
            ros::Rate loop_rate(10);
            while (ros::ok() && !action_completed[current_action]) {
                ros::spinOnce();
                loop_rate.sleep();
            }
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

