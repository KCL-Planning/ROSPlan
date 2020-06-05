#!/usr/bin/env python

import rospy
import importlib
import actionlib
from BaseActionInterface import BaseActionInterface

class ActionlibActionInterface(BaseActionInterface):

    # properties
    _has_default_topic = False
    _has_default_msg_type = False
    _has_default_goal = False

    def __init__(self, action_config):
        BaseActionInterface.__init__(self, action_config)

        # load properties from configuration
        self._has_default_topic = ("default_actionlib_topic" in action_config)
        self._has_default_msg_type = ("default_actionlib_msg_type" in action_config)
        self._has_default_goal = ("default_actionlib_goal" in action_config)


    def run(self, dispatch_msg):

        rospy.loginfo('KCL: ({}) Interfacing action {}'.format(rospy.get_name(), self._pddl_action_name))

        if self._has_default_topic:
            action_server = self.parse_config_string(self._action_config["default_actionlib_topic"], dispatch_msg)[0]

        if self._has_default_msg_type:
            goal_msg_type = self.parse_config_string(self._action_config["default_actionlib_msg_type"], dispatch_msg)[0]

        # TODO else, set by parameter value set
        # loop for each param set
            # if param set matches dispatch msg:
                # set action server, msg type, goal

        # import goal msg type
        i = goal_msg_type.find('/')
        pkg_name = goal_msg_type[:i]
        msg_name = goal_msg_type[i+1:]

        pkg = importlib.import_module(pkg_name+".msg")
        msg_action = getattr(pkg,msg_name+"Action")
        msg_goal = getattr(pkg,msg_name+"Goal")

        # create client and goal msg
        action_client = actionlib.SimpleActionClient(action_server, msg_action)
        action_client.wait_for_server() # TODO not wait infinitely
        goal_msg = msg_goal()
        
        # populate goal msg
        if self._has_default_goal:
            for param in self._action_config["default_actionlib_goal"]:
                value = self._action_config["default_actionlib_goal"][param]
                self.populate_goal_msg(goal_msg, param, value, dispatch_msg)
        # TODO use override if there is one

        # call service
        self._action_status[dispatch_msg.action_id] = self._status.ACTIVE
        def callback_lambda(status, result): self.action_finished_cb(dispatch_msg, status, result)
        action_client.send_goal(goal_msg, done_cb=callback_lambda)

    def action_finished_cb(self, dispatch_msg, status, result):
        self._action_status[dispatch_msg.action_id] = status
        print status, result

