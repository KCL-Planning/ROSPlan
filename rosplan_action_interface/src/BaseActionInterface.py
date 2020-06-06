#!/usr/bin/env python

import rospy
import actionlib
from abc import abstractmethod

# Base action interface to be exended into actionlib, service, FSM, etc.
# Uses actionlib states to describe status of actions.
class BaseActionInterface:

    _status = actionlib.GoalStatus
    _action_status = {}
    _action_name = None
    _action_config = None

    def __init__(self, action_config):
        self._action_name = action_config["name"]
        self._action_config = action_config

    def get_action_name(self):
        return self._action_name

    def get_plan_id(self,dispatch_msg):
        pass

    @staticmethod
    def params_match(config, pddl_params, dispatch_msg):
        for p in dispatch_msg.parameters:
            for i in range(len(pddl_params)):
                if p.key == pddl_params[i] and p.value != config["values"][i]:
                    return False
        return True

    @abstractmethod
    def run(self, dispatch_msg):
        pass

    #=====================#
    # msg parsing methods #
    #=====================#

    # populate a message with a parameter value
    def populate_goal_msg(self, goal_msg, param, value, dispatch_msg):
        param_string = self.parse_config_string(param, dispatch_msg)[0]
        value_string = self.parse_config_string(value, dispatch_msg)[0]
        p = eval("goal_msg." + param_string)
        if isinstance(p, str):
            exec("goal_msg." + param_string + " = \'" + value_string + "\'", {}, {'goal_msg': goal_msg})
        else:
            exec("goal_msg." + param_string + " = " + value_string, {}, {'goal_msg': goal_msg})

    # parse a string from the config and return the parsed
    # string. Inserts ROS and PDDL parameters where necessary.
    def parse_config_string(self, config_string, action_msg):

        # Bool and numeric values can be returned without parsing.
        if not isinstance(config_string, str):
            return str(config_string)

        depth = 0   
        return_string = ""

        i = 0
        while i < len(config_string):
            if config_string[i:i+12]=="($pddlparam ":
                param_key, length = self.parse_config_string(config_string[i+12:], action_msg)
                i = i + length + 12
                for p in action_msg.parameters:
                    if p.key == param_key:
                        return_string = return_string + p.value
            elif config_string[i:i+11]=="($rosparam ":
                param_key, length = self.parse_config_string(config_string[i+11:], action_msg)
                param_value = str(rospy.get_param(param_key))
                return_string = return_string + param_value
                i = i + length + 11
            elif config_string[i]==")":
                return return_string, i
            else:
                return_string = return_string + config_string[i]
            i = i + 1
        return return_string, len(config_string)

