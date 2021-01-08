#!/usr/bin/env python

import rospy
import actionlib
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest, KnowledgeUpdateServiceArray
from actionlib_msgs.msg import GoalStatus

from BaseActionInterface import BaseActionInterface
from ActionlibActionInterface import ActionlibActionInterface
from ServiceActionInterface import ServiceActionInterface
from FSMActionInterface import FSMActionInterface

# This class defines the action interface manager node. The node:
# - initialises a set of action interfaces according to config file;
# - subscribes to the PDDL action dispatch topic;
# - runs action execution through interfaces;
# - updates knowledge base with action effects;
# - publishes to PDDL action feedback.
class ActionInterfaceManager(object):

    # interfaces to manage
    _action_interfaces = {}

    def __init__(self):

        # knowledge base
        kb = rospy.get_param('~knowledge_base', 'knowledge_base')
        ss = '/' + kb + '/update_array'
        self.update_knowledge_client = rospy.ServiceProxy(ss, KnowledgeUpdateServiceArray)

        # load action interfaces from configuration file
        found_config = False
        if rospy.has_param('~actions'):
            self.cfg_actions = rospy.get_param('~actions')
            found_config = True
        if not found_config:
            rospy.logerr('KCL: ({}) Error: configuration file was not laoded.'.format(rospy.get_name()))
            rospy.signal_shutdown('Config not found')
            return

        self.parse_config()

        # publish to action feedback
        aft = rospy.get_param('~action_feedback_topic', 'default_feedback_topic')
        self.action_feedback_pub = rospy.Publisher(aft, ActionFeedback, queue_size=10)

        # subscribe to action dispatch
        adt = rospy.get_param('~action_dispatch_topic', 'default_dispatch_topic')
        rospy.Subscriber(adt, ActionDispatch, self.dispatch_callback, queue_size=10)

        rospy.loginfo('KCL: ({}) Ready to receive'.format(rospy.get_name()))
        self.run()

    # PDDL action dispatch callback
    def dispatch_callback(self, pddl_action_msg):

        if not pddl_action_msg.name in self._action_interfaces:
            # manager does not handle this PDDL action
            return

        # find and run action interface
        current_interface = self._action_interfaces[pddl_action_msg.name]
        current_interface.run(pddl_action_msg)

    def publish_feedback(self, action_id, msg):
        fb = ActionFeedback()
        fb.action_id = action_id
        fb.status = msg
        self.action_feedback_pub.publish(fb)

    # main management loop
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # iterate through interfaces
            for interface in self._action_interfaces.values():
                for action_id in interface._action_status.keys():

                    # send enabled feedback for newly interfaced actions
                    if interface._enable_flag[action_id]:
                        self.publish_feedback(action_id, 'action enabled')
                        interface._enable_flag[action_id] = False

                    # send feedback or status
                    if interface._action_status[action_id] == interface._status.SUCCEEDED:
                        rospy.loginfo('KCL: ({}) Reporting action complete: ({}) {}'.format(rospy.get_name(), action_id, interface._action_name))
                        self.publish_feedback(action_id, 'action achieved')
                        del interface._action_status[action_id]
                    elif interface._action_status[action_id] == interface._status.PREEMPTED:
                        rospy.loginfo('KCL: ({}) Reporting action cancelled: ({}) {}'.format(rospy.get_name(), action_id, interface._action_name))
                        self.publish_feedback(action_id, 'action cancelled')
                        del interface._action_status[action_id]
                    elif interface._action_status[action_id] == interface._status.ABORTED:
                        rospy.loginfo('KCL: ({}) Reporting action failed: ({}) {}'.format(rospy.get_name(), action_id, interface._action_name))
                        self.publish_feedback(action_id, 'action failed')
                        del interface._action_status[action_id]
                    elif interface._action_status[action_id] == interface._status.REJECTED:
                        rospy.loginfo('KCL: ({}) Reporting action rejected: ({}) {}'.format(rospy.get_name(), action_id, interface._action_name))
                        self.publish_feedback(action_id, 'action failed')
                        del interface._action_status[action_id]
                    else:
                        # PENDING=0; ACTIVE=1; PREEMPTING=6;
                        # RECALLING=7; RECALLED=8; LOST=9
                        pass
            rate.sleep()

    #==============#
    # YAML parsing #
    #==============#

    # parse YAML config and create action interfaces
    def parse_config(self):
        for action in self.cfg_actions:
            if action["interface_type"] == "actionlib":
                self.parse_actionlib(action)
            if action["interface_type"] == "service":
                self.parse_service(action)
            if action["interface_type"] == "fsm":
                self.parse_state_machine(action)

    # base case: parse actionlib interface
    def parse_actionlib(self, action_config):
        ai = ActionlibActionInterface(action_config)
        self._action_interfaces[ai.get_action_name()] = ai

    # base case: parse service interface
    def parse_service(self, action_config):
        ai = ServiceActionInterface(action_config)
        self._action_interfaces[ai.get_action_name()] = ai

    # parse fsm interface
    def parse_state_machine(self, action_config):
        ai = FSMActionInterface(action_config)
        self._action_interfaces[ai.get_action_name()] = ai


if __name__ == '__main__':
    rospy.init_node('RPStateMachine')
    aim = ActionInterfaceManager()
    #smm.run()
    rospy.spin()
