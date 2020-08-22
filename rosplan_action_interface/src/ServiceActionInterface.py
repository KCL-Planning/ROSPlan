#!/usr/bin/env python

import rospy
import importlib
from BaseActionInterface import BaseActionInterface
import threading
from rosplan_dispatch_msgs.msg import ActionFeedback

class ServiceActionInterface(BaseActionInterface):

    # properties
    _has_default_topic = False
    _has_default_msg_type = False
    _has_default_goal = False

    def __init__(self, action_config, parent_ai, action_feedback_pub):
        BaseActionInterface.__init__(self, action_config, parent_ai, action_feedback_pub)

        # load properties from configuration
        self._has_default_topic = ("default_service_topic" in action_config)
        self._has_default_msg_type = ("default_service_msg_type" in action_config)
        self._has_default_goal = ("default_service_goal" in action_config)

    def run(self, dispatch_msg):
        # Run on a new thread
        new_thread = threading.Thread(target=self.run_thread, args=(dispatch_msg,))
        new_thread.start()

    def run_thread(self, dispatch_msg):

        rospy.loginfo('KCL: ({}) Interfacing service {}'.format(rospy.get_name(), self._action_name))

        if self._has_default_topic:
            action_server = self.parse_config_string(self._action_config["default_service_topic"], dispatch_msg)[0]

        if self._has_default_msg_type:
            goal_msg_type = self.parse_config_string(self._action_config["default_service_msg_type"], dispatch_msg)[0]

        override_goal = None
        override_result = None
        if "pddl_parameters" in self._action_config:
            pddl_params = self._action_config["pddl_parameters"]
            for config in self._action_config["parameter_values"]:
                # Check parameter is correct
                if self.params_match(config, pddl_params, dispatch_msg):
                    if "service_topic" in config:
                        action_server = self.parse_config_string(config["service_topic"], dispatch_msg)[0]
                    if "service_msg_type" in config:
                        goal_msg_type = self.parse_config_string(config["service_msg_type"], dispatch_msg)[0]
                    if "service_goal" in config:
                        override_goal = config["service_goal"]
                    if "service_result" in config:
                        override_result = config["service_result"]

        # import goal msg type
        i = goal_msg_type.find('/')
        pkg_name = goal_msg_type[:i]
        msg_name = goal_msg_type[i+1:]

        pkg = importlib.import_module(pkg_name+".srv")
        srv = getattr(pkg,msg_name)
        req = getattr(pkg,msg_name+"Request")

        # create client and goal msg
        srv_proxy = rospy.ServiceProxy(action_server, srv)
        rospy.wait_for_service(action_server, timeout=10)
        request = req()

        # populate goal msg
        if self._has_default_goal:
            for param in self._action_config["default_service_goal"]:
                value = self._action_config["default_service_goal"][param]
                self.populate_goal_msg(request, param, value, dispatch_msg)

        if override_goal is not None:
            for param in override_goal:
                value = override_goal[param]
                self.populate_goal_msg(request, param, value, dispatch_msg)

        # call service
        # If succedded/ returned true
        result = srv_proxy(request)

        if result is not None:
            # Check results
            results_correct = True
            if override_result is not None:
                for param in override_result:
                    value = override_result[param]
                    if not self.check_result_msg(result, param, value, dispatch_msg):
                        results_correct = False
                        break

            if results_correct:
                # If this action interface has a parent interface
                if self._parent_ai is not None:
                    self._parent_ai.pass_child_action_finished_cb("succeeded", dispatch_msg)
                else:
                    # Otherwise, if this service is on the highest level, set the final status
                    rospy.loginfo(
                        'KCL: ({}) Plan {} Action {}: Server {} finished to goal state'.format(rospy.get_name(),
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
                # If the results are not correct
                # If this action interface has a parent interface
                if self._parent_ai is not None:
                    self._parent_ai.pass_child_action_finished_cb("failed", dispatch_msg)
                else:
                    # Otherwise, if this service is on the highest level, set the final status
                    rospy.loginfo(
                        'KCL: ({}) Plan {} Action {}: Server {} failed '.format(rospy.get_name(),
                                                                                dispatch_msg.plan_id,
                                                                                dispatch_msg.action_id,
                                                                                self.get_action_name()))
                    # Do not apply the end effects

                    # Publish feedback: action failure
                    fb = ActionFeedback()
                    fb.action_id = dispatch_msg.action_id
                    fb.plan_id = dispatch_msg.plan_id
                    fb.status = ActionFeedback.ACTION_FAILED
                    self._action_feedback_pub.publish(fb)
        else:
            # If failed/ returned false
            # If this action interface has a parent interface
            if self._parent_ai is not None:
                self._parent_ai.pass_child_action_finished_cb("failed", None, dispatch_msg)
            else:
                # Otherwise, if this service is on the highest level, set the final status
                rospy.loginfo(
                    'KCL: ({}) Plan {} Action {}: Server {} failed '.format(rospy.get_name(),
                                                                     dispatch_msg.plan_id,
                                                                     dispatch_msg.action_id,
                                                                     self.get_action_name()))
                # Do not apply the end effects

                # Publish feedback: action failure
                fb = ActionFeedback()
                fb.action_id = dispatch_msg.action_id
                fb.plan_id = dispatch_msg.plan_id
                fb.status = ActionFeedback.ACTION_FAILED
                self._action_feedback_pub.publish(fb)
