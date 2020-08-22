#!/usr/bin/env python

import rospy
import importlib
import actionlib
from BaseActionInterface import BaseActionInterface
import threading
from rosplan_dispatch_msgs.msg import ActionFeedback

class ActionlibActionInterface(BaseActionInterface):

    # properties
    _has_default_topic = False
    _has_default_msg_type = False
    _has_default_goal = False

    def __init__(self, action_config, parent_ai, action_feedback_pub):
        BaseActionInterface.__init__(self, action_config, parent_ai, action_feedback_pub)

        # load properties from configuration
        self._has_default_topic = ("default_actionlib_topic" in action_config)
        self._has_default_msg_type = ("default_actionlib_msg_type" in action_config)
        self._has_default_goal = ("default_actionlib_goal" in action_config)

    def __del__(self):
        print('Destructor of ActionLibAction.')

    def create_action_client(self, action_server, msg_action):
        action_client = actionlib.SimpleActionClient(action_server, msg_action)
        # Wait for server to be up
        while not rospy.is_shutdown():
            found = action_client.wait_for_server(rospy.Duration(10.0))
            if not found:
                rospy.logwarn('KCL: ({}) Could not contact action server {}, action {} cannot be executed'.format(
                    rospy.get_name(), action_server, self._action_name))
            else:
                return action_client

    def run(self, dispatch_msg):
        # Run on a new thread
        new_thread = threading.Thread(target=self.run_thread, args=(dispatch_msg,))
        new_thread.start()

    def run_thread(self, dispatch_msg):
        self._action_interface_status = 1

        rospy.loginfo('KCL: ({}) Interfacing action {}'.format(rospy.get_name(), self._action_name))

        if self._has_default_topic:
            action_server = self.parse_config_string(self._action_config["default_actionlib_topic"], dispatch_msg)[0]

        if self._has_default_msg_type:
            goal_msg_type = self.parse_config_string(self._action_config["default_actionlib_msg_type"], dispatch_msg)[0]

        override_goal = None
        override_result = None
        if "pddl_parameters" in self._action_config:
            pddl_params = self._action_config["pddl_parameters"]
            for config in self._action_config["parameter_values"]:
                # Check parameter is correct
                if self.params_match(config, pddl_params, dispatch_msg):
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
        action_client = self.create_action_client(action_server, msg_action)
        goal_msg = msg_goal()
        
        # populate goal msg
        if self._has_default_goal:
            for param in self._action_config["default_actionlib_goal"]:
                value = self._action_config["default_actionlib_goal"][param]
                self.populate_goal_msg(goal_msg, param, value, dispatch_msg)

        if override_goal is not None:  # This will override whatever has been set by the default goal
            for param in override_goal:
                value = override_goal[param]
                self.populate_goal_msg(goal_msg, param, value, dispatch_msg)

        # call service
        rospy.loginfo(
            'KCL: ({}) Plan {} Action {}: Action server {} on thread {}'.format(rospy.get_name(),
                                                                   dispatch_msg.plan_id,
                                                                   dispatch_msg.action_id,
                                                                   action_server,
                                                                   threading.currentThread()))

        #def callback_lambda(status, result): self.action_finished_cb(dispatch_msg, status, result)
        callback_lambda = lambda status, result: self.action_finished_cb(override_result, dispatch_msg, status, result)

        action_client.send_goal(goal_msg, done_cb=callback_lambda)
        action_client.wait_for_result(rospy.Duration.from_sec(60.0))

    def action_finished_cb(self, override_result, dispatch_msg, status, result):
        rospy.loginfo('KCL: ({}) Plan {} Action {}: Action_finished_cb call on thread {}'.format(rospy.get_name(),
                                                                                    dispatch_msg.plan_id,
                                                                                    dispatch_msg.action_id,
                                                                                    threading.currentThread()))

        # Check results
        results_correct = True
        if override_result is not None:
            for param in override_result:
                value = override_result[param]
                if not self.check_result_msg(result, param, value, dispatch_msg):
                    results_correct = False
                    break

        if status == actionlib.GoalStatus.SUCCEEDED and results_correct:
            # If this action interface has a parent interface
            if self._parent_ai is not None:
                self._parent_ai.pass_child_action_finished_cb("succeeded", dispatch_msg)
            else:
                # Otherwise, if this actionlib is on the highest level, set the final status
                rospy.loginfo(
                    'KCL: ({}) Plan {} Action {}: ActionLib {} finished to goal state'.format(rospy.get_name(),
                                                                                       dispatch_msg.plan_id,
                                                                                       dispatch_msg.action_id,
                                                                                       self.get_action_name()))
                # Apply the end effects
                self._kb_link.kb_apply_action_effects(dispatch_msg, 1)

                # Publish feedback: action succeeded to goal state
                fb = ActionFeedback()
                fb.action_id = dispatch_msg.action_id
                fb.plan_id = dispatch_msg.plan_id
                fb.status = ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE
                self._action_feedback_pub.publish(fb)

        else:
            # If this action interface has a parent interface
            if self._parent_ai is not None:
                self._parent_ai.pass_child_action_finished_cb("failed", dispatch_msg)
            else:
                # Otherwise, if this actionlib is on the highest level, set the final status
                # Do not apply the end effects

                # Publish feedback: action failed
                fb = ActionFeedback()
                fb.action_id = dispatch_msg.action_id
                fb.plan_id = dispatch_msg.plan_id
                fb.status = ActionFeedback.ACTION_FAILED
                self._action_feedback_pub.publish(fb)
