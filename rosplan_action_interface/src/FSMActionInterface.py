#!/usr/bin/env python3

import rospy, copy
import actionlib
from BaseActionInterface import BaseActionInterface
from ActionlibActionInterface import ActionlibActionInterface
from ServiceActionInterface import ServiceActionInterface
import threading
from rosplan_dispatch_msgs.msg import ActionFeedback

# ToDo: Implement mechanism to check cyclic execution

class Transition(object):
    def __init__(self, state_name, effects):
        self._to_state = state_name
        self._effects = ''  # ToDo: parse pddl expression for effects

    def get_to_state(self):
        return self._to_state

    def get_effects(self):
        return self._effects


class State(object):
    def __init__(self, state_name, action_interface, transitions):

        # Name
        self._state_name = state_name

        # Action Interface
        self._action_interface = action_interface

        # Transitions
        self._transitions = transitions

    def get_state_name(self):
        return self._state_name

    def get_action_interface(self):
        return self._action_interface

    def get_transition(self, transition_name, counter):
        rospy.loginfo("transition_name = {}; counter = {}".format(transition_name, counter))
        return self._transitions[transition_name][counter]


class FSMActionInterface(BaseActionInterface):

    def __init__(self, action_config, action_feedback_pub):
        BaseActionInterface.__init__(self, action_config, action_feedback_pub)
        # 'Static' data
        self._states = {}

        # 'Dynamic' data
        self._running_state = {}
        self._transition_value = {}
        self._to_start_state = {}
        self._transitions_counters = {}

        # TODO: Where and how to clear the dynamic data after the actions were executed

        # Parse states
        for state_config in action_config["states"]:
            if state_config["interface_type"] == "actionlib":
                ai = ActionlibActionInterface(state_config)
                transitions = self.parse_transitions(state_config)

            if state_config["interface_type"] == "service":
                ai = ServiceActionInterface(state_config)
                transitions = self.parse_transitions(state_config)

            if state_config["interface_type"] == "fsm":
                ai = FSMActionInterface(state_config)
                transitions = self.parse_transitions(state_config)

            self._states[state_config["name"]] = State(state_config["name"], ai, transitions)

    # TODO replace with FSM logic
    def pass_child_action_finished_cb(self, transition_value, dispatch_msg):
        self._transition_value[dispatch_msg.plan_id][dispatch_msg.action_id] = transition_value

    def run(self, dispatch_msg):
        # Run on a new thread
        new_thread = threading.Thread(target=self.run_thread, args=(dispatch_msg,))
        new_thread.start()

    def run_thread(self, dispatch_msg):

        rospy.loginfo('KCL: ({}) Plan {} Action {}: State machine {} running on thread {}'.format(rospy.get_name(),
                                                                                     dispatch_msg.plan_id,
                                                                                     dispatch_msg.action_id,
                                                                                     self.get_action_name(),
                                                                                     threading.currentThread()))

        # Extend the dictionaries if needed
        if dispatch_msg.plan_id not in self._running_state:
            self._running_state[dispatch_msg.plan_id] = {}
            self._transition_value[dispatch_msg.plan_id] = {}
            self._to_start_state[dispatch_msg.plan_id] = {}
            self._transitions_counters[dispatch_msg.plan_id] = {}
        if dispatch_msg.action_id not in self._running_state[dispatch_msg.plan_id]:

            self._running_state[dispatch_msg.plan_id][dispatch_msg.action_id] = None
            self._transition_value[dispatch_msg.plan_id][dispatch_msg.action_id] = None
            self._to_start_state[dispatch_msg.plan_id][dispatch_msg.action_id] = False
            self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id] = {}
            self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["succeeded"] = 0
            self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["failed"] = 0

        # Start running the first basic action
        self._running_state[dispatch_msg.plan_id][dispatch_msg.action_id] = self._states["ba1"]
        action_interface = self._running_state[dispatch_msg.plan_id][dispatch_msg.action_id].get_action_interface()
        action_interface.run(dispatch_msg)

        # Execute the fsm
        fsm_execution_completed = False
        while not fsm_execution_completed:

            transition_value = self._transition_value[dispatch_msg.plan_id][dispatch_msg.action_id]

            if not transition_value == None:
                # Reset the transition value
                self._transition_value[dispatch_msg.plan_id][dispatch_msg.action_id] = None

                # Based on the passed transition value get the transition and modify the counters accordingly
                transition = None
                if transition_value == "succeeded":
                    transition_counter = self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["succeeded"]
                    transition = self._running_state[dispatch_msg.plan_id][dispatch_msg.action_id].get_transition("succeeded", transition_counter)
                    self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["succeeded"] += 1
                    self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["failed"] = 0

                if transition_value == "failed":
                    transition_counter = self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["failed"]
                    transition = self._running_state[dispatch_msg.plan_id][dispatch_msg.action_id].get_transition("failed", transition_counter)
                    self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["failed"] += 1
                    self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["succeeded"] = 0

                if transition.get_to_state().find("ba") != -1:

                    # If this fsm is on the highest level and it is transited for the first time to a reverse state
                    if transition.get_to_state().find("reverse") != -1 and \
                         not self._to_start_state[dispatch_msg.plan_id][dispatch_msg.action_id] and \
                         self._parent_ai is None:

                        # Mark the execution to start state
                        self._to_start_state[dispatch_msg.plan_id][dispatch_msg.action_id] = True

                        # Publish feedback
                        fb = ActionFeedback()
                        fb.action_id = dispatch_msg.action_id
                        fb.plan_id = dispatch_msg.plan_id
                        fb.status = ActionFeedback.ACTION_DISPATCHED_TO_START_STATE
                        self._action_feedback_pub.publish(fb)

                    rospy.loginfo('KCL: ({}) Plan {} Action {}: Transition to state {}'.format(rospy.get_name(),
                                                                                               dispatch_msg.plan_id,
                                                                                               dispatch_msg.action_id,
                                                                                               transition.get_to_state()))

                    # Update the running state and get the names of the old and new states
                    old_running_state_name = self._running_state[dispatch_msg.plan_id][
                        dispatch_msg.action_id].get_state_name()
                    self._running_state[dispatch_msg.plan_id][dispatch_msg.action_id] = self._states[
                        transition.get_to_state()]
                    new_running_state_name = self._running_state[dispatch_msg.plan_id][
                        dispatch_msg.action_id].get_state_name()

                    # Reset the transition counters for the next state only if transited to another state
                    if old_running_state_name != new_running_state_name:
                        self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["succeeded"] = 0
                        self._transitions_counters[dispatch_msg.plan_id][dispatch_msg.action_id]["failed"] = 0

                    # Run the action interface of the new state
                    action_interface = self._running_state[dispatch_msg.plan_id][
                        dispatch_msg.action_id].get_action_interface()
                    action_interface.run(dispatch_msg)

                # If this function was called from the goal state of the fsm
                elif transition.get_to_state() == "goal_state":

                    rospy.loginfo('KCL: ({}) Plan {} Action {}: ActionLib {} finished to goal state'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))
                    self._action_status[plan_action_id] = ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE
                    fsm_execution_completed = True

                # If this function was called from the start state of the fsm
                elif transition.get_to_state() == "start_state":

                    rospy.loginfo('KCL: ({}) Plan {} Action {}: ActionLib {} finished to start state'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))
                    self._action_status[plan_action_id] = ActionFeedback.ACTION_SUCCEEDED_TO_START_STATE
                    fsm_execution_completed = True

                    # undo the start effects
                    self._kb_link.kb_undo_action_effects(dispatch_msg, RPKnowledgeBaseLink.AT_START)

                # If this function was called from an error state of the fsm
                elif transition.get_to_state() == "error_state":
                    rospy.logwarn('KCL: ({}) Plan {} Action {}: State machine {} error. Human intervention needed'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))
                else:
                    rospy.logwarn('KCL: ({}) Plan {} Action {}: State machine {} error: Transition to unknown state'.format(rospy.get_name(), dispatch_msg.plan_id, dispatch_msg.action_id, self.get_action_name()))


    def parse_transitions(self, state):
        # Create a dictionary with all transition types, where for each of them an array with transitions is parsed
        # from the input yaml file
        transitions = {}
        transitions["succeeded"] = []
        transitions["failed"] = []
        for transition in state["transitions"]["succeeded"]:
            transitions["succeeded"].append(Transition(transition["to_state"], transition["effects"]))
        for transition in state["transitions"]["failed"]:
            transitions["failed"].append(Transition(transition["to_state"], transition["effects"]))
        return transitions
