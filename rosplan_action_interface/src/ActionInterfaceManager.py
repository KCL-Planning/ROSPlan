#!/usr/bin/env python3

import rospy
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback

from BaseActionInterface import BaseActionInterface
from ActionlibActionInterface import ActionlibActionInterface
from ServiceActionInterface import ServiceActionInterface
from FSMActionInterface import FSMActionInterface
from RPKnowledgeBaseLink import RPKnowledgeBaseLink

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

        # knowledge base link
        self._kb_link = RPKnowledgeBaseLink()

        # load action interfaces from configuration file
        found_config = False
        if rospy.has_param('~actions'):
            self.cfg_actions = rospy.get_param('~actions')
            found_config = True
        if not found_config:
            rospy.logerr('KCL: ({}) Error: configuration file was not laoded.'.format(rospy.get_name()))
            rospy.signal_shutdown('Config not found')
            return

        # feedback
        aft = rospy.get_param('~action_feedback_topic', 'default_feedback_topic')
        self._action_feedback_pub = rospy.Publisher(aft, ActionFeedback, queue_size=10)

        # subscribe to action dispatch
        adt = rospy.get_param('~action_dispatch_topic', 'default_dispatch_topic')
        rospy.Subscriber(adt, ActionDispatch, self.dispatch_callback, queue_size=10)

        self.parse_config()

        rospy.loginfo('KCL: ({}) Ready to receive'.format(rospy.get_name()))
        self.run()

    #======================#
    # PDDL action messages #
    #======================#

    # PDDL action dispatch callback
    def dispatch_callback(self, pddl_action_msg):

        if not pddl_action_msg.name in self._action_interfaces:
            # manager does not handle this PDDL action
            return

        # Publish feedback: action enabled
        self.publish_feedback(pddl_action_msg.plan_id, pddl_action_msg.action_id, ActionFeedback.ACTION_DISPATCHED_TO_GOAL_STATE)

        # Set the start effects
        self._kb_link.kb_apply_action_effects(pddl_action_msg, RPKnowledgeBaseLink.AT_START)

        # find and run action interface
        current_interface = self._action_interfaces[pddl_action_msg.name]
        current_interface.run(pddl_action_msg)

    # PDDL action feedback
    def publish_feedback(self, plan_id, action_id, status):
        fb = ActionFeedback()
        fb.action_id = action_id
        fb.plan_id = plan_id
        fb.status = status
        self._action_feedback_pub.publish(fb)

    #==================#
    # interface status #
    #==================#

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
                
            # iterate through interfaces and send feedback
            for interface in self._action_interfaces.values():
                for act in interface._action_status.keys():

                    # action successful
                    if interface._action_status[act] == ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE:
                        # Apply the end effects
                        self._kb_link.kb_apply_action_effects(interface._action_dispatch_msg[act], RPKnowledgeBaseLink.AT_END)

                    # action completed (achieved or failed)
                    if interface._action_status[act] == ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE or interface._action_status[act] == ActionFeedback.ACTION_FAILED:
                        rospy.loginfo('KCL: ({}) Reporting action complete: {} {}'.format(rospy.get_name(), act, interface._action_name))
                        # publish feedback msg
                        self.publish_feedback(act[0], act[1], interface._action_status[act])
                        # remove completed action data from interface
                        interface.clean_action(act)

            rate.sleep()

    #==============#
    # YAML parsing #
    #==============#

    # parse YAML config and create action interfaces
    def parse_config(self):
        for action in self.cfg_actions:
            if action["interface_type"] == "actionlib":
                self._action_interfaces[action["name"]] = ActionlibActionInterface(action)
            if action["interface_type"] == "service":
                self._action_interfaces[action["name"]] = ServiceActionInterface(action)
            if action["interface_type"] == "fsm":
                self._action_interfaces[action["name"]] = FSMActionInterface(action)

if __name__ == '__main__':
    rospy.init_node('RPActionInterfaceManager')
    aim = ActionInterfaceManager()
    rospy.spin()
