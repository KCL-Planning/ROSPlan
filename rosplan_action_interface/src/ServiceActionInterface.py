#!/usr/bin/env python

import rospy
import importlib
from BaseActionInterface import BaseActionInterface


class ServiceActionInterface(BaseActionInterface):

    # properties
    _has_default_topic = False
    _has_default_msg_type = False
    _has_default_goal = False

    def __init__(self, action_config):
        BaseActionInterface.__init__(self, action_config)

        # load properties from configuration
        self._has_default_topic = ("default_service_topic" in action_config)
        self._has_default_msg_type = ("default_service_msg_type" in action_config)
        self._has_default_goal = ("default_service_goal" in action_config)

    def run(self, dispatch_msg):

        rospy.loginfo('KCL: ({}) Interfacing action {}'.format(rospy.get_name(), self._action_name))

        if self._has_default_topic:
            action_server = self.parse_config_string(self._action_config["default_service_topic"], dispatch_msg)[0]

        if self._has_default_msg_type:
            goal_msg_type = self.parse_config_string(self._action_config["default_service_msg_type"], dispatch_msg)[0]

        override_goal = None
        if "pddl_parameters" in self._action_config:
            pddl_params = self._action_config["pddl_parameters"]
            for config in self._action_config["parameter_values"]:
                # Check parameter is correct
                if self.params_match(config, pddl_params, dispatch_msg):
                    override_goal = config["service_goal"]
                    if "service_topic" in config:
                        action_server = self.parse_config_string(config["service_topic"], dispatch_msg)[0]
                    if "service_msg_type" in config:
                        goal_msg_type = self.parse_config_string(config["service_msg_type"], dispatch_msg)[0]

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
        self._action_status[dispatch_msg.action_id] = self._status.ACTIVE
        result = srv_proxy(request)
        self._action_status[dispatch_msg.action_id] = self._status.SUCCEEDED

