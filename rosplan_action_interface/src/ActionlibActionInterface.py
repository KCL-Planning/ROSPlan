#!/usr/bin/env python3

import rospy
import importlib
import actionlib
from BaseActionInterface import BaseActionInterface
from rosplan_dispatch_msgs.msg import ActionFeedback

class ActionlibActionInterface(BaseActionInterface):

    def __init__(self, action_config):
        BaseActionInterface.__init__(self, action_config)

        # load properties from configuration
        self._has_default_topic = ("default_actionlib_topic" in action_config)
        self._has_default_msg_type = ("default_actionlib_msg_type" in action_config)
        self._has_default_goal = ("default_actionlib_goal" in action_config)
        self._has_default_result = ("default_actionlib_result" in action_config)

        self._action_client = {}


    def clean_action(self, plan_action_id):
        BaseActionInterface.clean_action(self, plan_action_id)
        del self._action_client[plan_action_id]

    def create_action_client(self, action_server, msg_action):
        action_client = actionlib.SimpleActionClient(action_server, msg_action)
        # Wait for server to be up
        while not rospy.is_shutdown():
            found = action_client.wait_for_server(rospy.Duration(10.0))
            if not found:
                rospy.logwarn('KCL: ({}) Could not contact action server {}, action {} cannot be executed'.format(rospy.get_name(), action_server, self._action_name))
            else:
                return action_client
        return None

    def run(self, dispatch_msg):

        rospy.loginfo('KCL: ({}) Interfacing action {}'.format(rospy.get_name(), self._action_name))

        # set action status
        plan_action_id = (dispatch_msg.plan_id, dispatch_msg.action_id)
        self._action_dispatch_msg[plan_action_id] = dispatch_msg
        self._action_status[plan_action_id] = ActionFeedback.ACTION_ENABLED

        # load default configuration for goal msg
        if self._has_default_topic:
            action_server = self.parse_config_string(self._action_config["default_actionlib_topic"], dispatch_msg)[0]
        if self._has_default_msg_type:
            goal_msg_type = self.parse_config_string(self._action_config["default_actionlib_msg_type"], dispatch_msg)[0]

        # load parameter-specific configuration for goal msg
        override_goal = None
        override_result = None
        if "pddl_parameters" in self._action_config:
            pddl_params = self._action_config["pddl_parameters"]
            for config in self._action_config["parameter_values"]:
                # Check parameter is correct
                if self.params_match(config, pddl_params, dispatch_msg):
                    override_goal = config["actionlib_goal"]
                    if "actionlib_topic" in config:
                        action_server = self.parse_config_string(config["actionlib_topic"], dispatch_msg)[0]
                    if "actionlib_msg_type" in config:
                        goal_msg_type = self.parse_config_string(config["actionlib_msg_type"], dispatch_msg)[0]
                    if "actionlib_goal" in config:
                        override_goal = config["actionlib_goal"]
                    if "actionlib_result" in config:
                        override_result = config["actionlib_result"]

        # import goal msg type
        i = goal_msg_type.find('/')
        pkg_name = goal_msg_type[:i]
        msg_name = goal_msg_type[i+1:]

        pkg = importlib.import_module(pkg_name+".msg")
        msg_action = getattr(pkg,msg_name+"Action")
        msg_goal = getattr(pkg,msg_name+"Goal")

        # create client and goal msg
        self._action_client[plan_action_id] = self.create_action_client(action_server, msg_action)
        goal_msg = msg_goal()
        if self._action_client[plan_action_id] == None:
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
            return
        
        # populate goal msg
        if self._has_default_goal:
            for param in self._action_config["default_actionlib_goal"]:
                value = self._action_config["default_actionlib_goal"][param]
                self.populate_goal_msg(goal_msg, param, value, dispatch_msg)

        if override_goal is not None:
            for param in override_goal:
                value = override_goal[param]
                self.populate_goal_msg(goal_msg, param, value, dispatch_msg)

        # call simple action client
        rospy.loginfo('KCL: ({}) Plan {} Action {}: Calling action server {}'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, action_server))
        callback_lambda = lambda status, result: self.action_finished_cb(override_result, dispatch_msg, status, result)
        self._action_client[plan_action_id].send_goal(goal_msg, done_cb=callback_lambda)


    def action_finished_cb(self, override_result, dispatch_msg, status, result):

        # check default expected results
        results_correct = True
        if self._has_default_result:
            for param in self._action_config["default_actionlib_result"]:
                if not override_result or not param in override_result:
                    value = self._action_config["default_actionlib_result"][param]
                    results_correct = self.check_result_msg(result, param, value, dispatch_msg)
                    if not results_correct:
                        break

        # check override expected results
        if results_correct and override_result is not None:
            for param in override_result:
                value = override_result[param]
                results_correct = self.check_result_msg(result, param, value, dispatch_msg)
                if not results_correct:
                    break


        plan_action_id = (dispatch_msg.plan_id, dispatch_msg.action_id)
        if status == actionlib.GoalStatus.SUCCEEDED and results_correct:
            rospy.loginfo('KCL: ({}) Plan {} Action {}: ActionLib {} finished to goal state'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE
        else:
            rospy.loginfo('KCL: ({}) Plan {} Action {}: ActionLib {} failed'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
