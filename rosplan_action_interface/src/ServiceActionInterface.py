#!/usr/bin/env python3

import rospy
import importlib
import threading
from BaseActionInterface import BaseActionInterface
from rosplan_dispatch_msgs.msg import ActionFeedback

class ServiceActionInterface(BaseActionInterface):

    def __init__(self, action_config):
        BaseActionInterface.__init__(self, action_config)

        # load properties from configuration
        self._has_default_topic = ("default_service" in action_config)
        self._has_default_msg_type = ("default_service_type" in action_config)
        self._has_default_request = ("default_service_request" in action_config)
        self._has_default_response = ("default_service_response" in action_config)

    def run(self, dispatch_msg):
        # Run on a new thread
        new_thread = threading.Thread(target=self.run_thread, args=(dispatch_msg,))
        new_thread.start()

    def run_thread(self, dispatch_msg):

        rospy.loginfo('KCL: ({}) Interfacing service {}'.format(rospy.get_name(), self._action_name))

        # set action status
        plan_action_id = (dispatch_msg.plan_id, dispatch_msg.action_id)
        self._action_dispatch_msg[plan_action_id] = dispatch_msg
        self._action_status[plan_action_id] = ActionFeedback.ACTION_ENABLED

        # load default configuration for request
        if self._has_default_topic:
            action_server = self.parse_config_string(self._action_config["default_service"], dispatch_msg)[0]

        if self._has_default_msg_type:
            goal_msg_type = self.parse_config_string(self._action_config["default_service_type"], dispatch_msg)[0]

        override_request = None
        override_result = None
        if "pddl_parameters" in self._action_config:
            pddl_params = self._action_config["pddl_parameters"]
            for config in self._action_config["parameter_values"]:
                # Check parameter is correct
                if self.params_match(config, pddl_params, dispatch_msg):
                    if "service" in config:
                        action_server = self.parse_config_string(config["service"], dispatch_msg)[0]
                    if "service_type" in config:
                        goal_msg_type = self.parse_config_string(config["service_type"], dispatch_msg)[0]
                    if "service_request" in config:
                        override_request = config["service_request"]
                    if "service_response" in config:
                        override_result = config["service_response"]

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
        if self._has_default_request:
            for param in self._action_config["default_service_request"]:
                value = self._action_config["default_service_request"][param]
                self.populate_goal_msg(request, param, value, dispatch_msg)

        if override_request:
            for param in override_request:
                value = override_request[param]
                self.populate_goal_msg(request, param, value, dispatch_msg)

        try: 
            # call service
            result = srv_proxy(request)
        except rospy.ServiceException as exc:
            rospy.logwarn('KCL: ({}) Plan {} Action {}: Service {} failed to call service'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
            return

        if result:

            # check default expected results
            results_correct = True
            if self._has_default_response:
                for param in self._action_config["default_service_response"]:
                    if not override_result or not param in override_result:
                        value = self._action_config["default_service_response"][param]
                        results_correct = self.check_result_msg(result, param, value, dispatch_msg)
                        if not results_correct:     
                            break

            # check override expected results
            if results_correct and override_result:
                for param in override_result:
                    value = override_result[param]
                    results_correct = self.check_result_msg(result, param, value, dispatch_msg)
                    if not results_correct:
                        break

            if results_correct:
                rospy.loginfo('KCL: ({}) Plan {} Action {}: Service {} finished to goal state'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))
                self._action_status[plan_action_id] = ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE
            else:
                rospy.loginfo('KCL: ({}) Plan {} Action {}: Service {} failed'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))
                self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED

        else:
            rospy.loginfo('KCL: ({}) Plan {} Action {}: Service {} failed'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))
            self._action_status[plan_action_id] = ActionFeedback.ACTION_FAILED
