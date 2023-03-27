#!/usr/bin/env python3

import os
import rospy
import rospkg
import sys

from itertools import product
from string import join, split

from std_msgs.msg import *
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import *
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

from python_qt_binding.QtCore import Qt, QTimer

class PlanViewWidget:

    # plan view
    _column_names = ['action_id', 'dispatch_time', 'action_name', 'duration', 'status']
    _action_list = []
    _status_list = {}
    _predicate_param_type_list = {}
    _predicate_param_label_list = {}

    # model view
    _type_list = []
    _goal_list = {}
    _fact_list = {}

    def __init__(self):

        # Create QWidget
        self.setObjectName('ROSPlanGraphUpdater')

        # init and start update timers
        self._timer_refresh_plan = QTimer(self)
        self._timer_refresh_plan.timeout.connect(self.refresh_plan)

        rospy.Subscriber("/kcl_rosplan/plan", CompletePlan, self.plan_callback)
        rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, self.action_feedback_callback)

    def start(self):
        self._timer_refresh_plan.start(1000)
        self._timer_refresh_goals.start(10000)

    """
    updating plan view
    """
    def refresh_plan(self):
        expanded_list = []
        root = self.planView.invisibleRootItem()
        child_count = root.childCount()
        for i in range(child_count):
            item = root.child(i)
            if item.isExpanded():
                expanded_list.append(item.text(self._column_index['action_id']))
        self.planView.clear()
        for action in self._action_list:
            item = QTreeWidgetItem(self.planView)
            item.setText(self._column_index['action_id'], str(action.action_id))
            item.setText(self._column_index['dispatch_time'], str(action.dispatch_time))
            item.setText(self._column_index['duration'], str(action.duration))
            item.setText(self._column_index['status'], self._status_list.get(str(action.action_id),"-"))
            action_name = '(' + action.name
            for keyval in action.parameters:
                param = QTreeWidgetItem(item)
                param.setText(self._column_index['action_id'], '')
                param.setText(self._column_index['action_name'], '- ' + keyval.key + ': ' + keyval.value)
                param.setText(self._column_index['dispatch_time'], '')
                param.setText(self._column_index['duration'], '')
                param.setText(self._column_index['status'], '')
                action_name = action_name + ' ' + keyval.value
            item.setText(self._column_index['action_name'], action_name +')')
            if str(action.action_id) in expanded_list:
                item.setExpanded(True)

    """
    callback for complete_plan
    """
    def plan_callback(self, data):
        self._action_list = data.plan

    """
    callback for action_feedback
    """
    def action_feedback_callback(self, data):
        self._status_list[str(data.action_id)] = data.status
        self.refresh_model()
