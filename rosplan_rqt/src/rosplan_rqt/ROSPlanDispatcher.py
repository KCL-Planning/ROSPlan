#!/usr/bin/env python3

import os
import rospy
import rospkg
import sys

from itertools import product

from std_msgs.msg import *
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import *
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
if QT_BINDING_VERSION.startswith('4'):
    from python_qt_binding.QtGui import QHeaderView, QIcon, QTreeWidgetItem, QListWidgetItem, QWidget
else:
    from python_qt_binding.QtWidgets import QHeaderView, QTreeWidgetItem, QListWidgetItem, QWidget
    from python_qt_binding.QtGui import QIcon

class PlanViewWidget(QWidget):

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

    def __init__(self, plugin=None):
        super(PlanViewWidget, self).__init__()

        # Create QWidget
        ui_file = os.path.join(rospkg.RosPack().get_path('rosplan_rqt'), 'resource', 'dispatcher.ui')
        loadUi(ui_file, self)
        self.setObjectName('ROSPlanDispatcherUI')

        rospy.loginfo('Waiting for rosplan_knowledge_base services to become available (5 sec timeout)')

        # populate goal combo boxes
        try:
            rospy.wait_for_service('rosplan_knowledge_base/domain/predicates', 5.0)
            predicates_client = rospy.ServiceProxy('rosplan_knowledge_base/domain/predicates', GetDomainAttributeService)
            resp = predicates_client()
            for pred in resp.items:
                self.goalNameComboBox.addItem(pred.name)
                self.factNameComboBox.addItem(pred.name)
                param_list = []
                label_list = []
                for param in pred.typed_parameters:
                    param_list.append(param.value)
                    label_list.append(param.key)
                self._predicate_param_type_list[pred.name] = param_list
                self._predicate_param_label_list[pred.name] = label_list
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')

        self._handle_goal_name_changed(0)
        self._handle_fact_name_changed(0)

        # populate type combo box
        rospy.wait_for_service('rosplan_knowledge_base/domain/types')
        try:
            type_client = rospy.ServiceProxy('rosplan_knowledge_base/domain/types', GetDomainTypeService)
            resp = type_client()
            for typename in resp.types:
                self.typeComboBox.addItem(typename)
                self._type_list.append(typename)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')

        # connect components
        self.planButton.clicked[bool].connect(self._handle_plan_clicked)
        self.pauseButton.clicked[bool].connect(self._handle_pause_clicked)
        self.cancelButton.clicked[bool].connect(self._handle_cancel_clicked)
        self.addInstanceButton.clicked[bool].connect(self._handle_add_instance_clicked)
        self.removeGoalButton.clicked[bool].connect(self._handle_remove_goal_clicked)
        self.removeFactButton.clicked[bool].connect(self._handle_remove_fact_clicked)
        self.addGoalButton.clicked[bool].connect(self._handle_add_goal_clicked)
        self.addFactButton.clicked[bool].connect(self._handle_add_fact_clicked)
        self.goalNameComboBox.currentIndexChanged[int].connect(self._handle_goal_name_changed)
        self.factNameComboBox.currentIndexChanged[int].connect(self._handle_fact_name_changed)

        self._plugin = plugin
        self.planView.sortByColumn(0, Qt.AscendingOrder)
        header = self.planView.header()
        if QT_BINDING_VERSION.startswith('4'):
            header.setResizeMode(QHeaderView.ResizeToContents)
        else:
            header.setSectionResizeMode(QHeaderView.ResizeToContents)

        # setup plan view columns
        self._column_index = {}
        for column_name in self._column_names:
            self._column_index[column_name] = len(self._column_index)

        # init and start update timers
        self._timer_refresh_plan = QTimer(self)
        self._timer_refresh_plan.timeout.connect(self.refresh_plan)
        self._timer_refresh_goals = QTimer(self)
        self._timer_refresh_goals.timeout.connect(self.refresh_model)

        rospy.Subscriber("/kcl_rosplan/plan", CompletePlan, self.plan_callback)
        rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, self.action_feedback_callback)
        rospy.Subscriber("/kcl_rosplan/system_state", String, self.system_status_callback)
        self._plan_pub = rospy.Publisher('/kcl_rosplan/planning_commands', String, queue_size=10)

        self.refresh_model()

    def start(self):
        self._timer_refresh_plan.start(1000)
        self._timer_refresh_goals.start(10000)

    """
    updating goal and model view
    """
    def refresh_model(self):
        # goals
        rospy.wait_for_service('rosplan_knowledge_base/state/goals')
        selected_list = []
        for item in self.goalView.selectedItems():
            selected_list.append(item.text())
        try:
            goals_client = rospy.ServiceProxy('rosplan_knowledge_base/state/goals', GetAttributeService)
            resp = goals_client('')
            self.goalView.clear()
            self._goal_list.clear()
            for goal in resp.attributes:
                item = QListWidgetItem(self.goalView)
                goalText = '(' + goal.attribute_name
                for keyval in goal.values:
                     goalText = goalText + ' ' + keyval.value
                goalText = goalText + ')'
                item.setText(goalText)
                self._goal_list[goalText] = goal
                if goalText in selected_list:
                    item.setSelected(True)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')
        # facts and functions
        rospy.wait_for_service('rosplan_knowledge_base/state/propositions')
        selected_list = []
        for item in self.modelView.selectedItems():
            selected_list.append(item.text())
        try:
            model_client = rospy.ServiceProxy('rosplan_knowledge_base/state/propositions', GetAttributeService)
            resp = model_client('')
            self.modelView.clear()
            self._fact_list.clear()
            for attribute in resp.attributes:
                attributeText = ''
                item = QListWidgetItem(self.modelView)
                if attribute.is_negative:
                    attributeText = '(not '
                attributeText = attributeText + '(' + attribute.attribute_name
                for keyval in attribute.values:
                     attributeText = attributeText + ' ' + keyval.value
                attributeText = attributeText + ')'
                if attribute.is_negative:
                    attributeText = attributeText + ')'
                item.setText(attributeText)
                self._fact_list[attributeText] = attribute
                if attributeText in selected_list:
                    item.setSelected(True)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')
        # instances
        expanded_list = []
        root = self.instanceView.invisibleRootItem()
        child_count = root.childCount()
        for i in range(child_count):
            item = root.child(i)
            if item.isExpanded():
                expanded_list.append(item.text(0))
        self.instanceView.clear()
        for typename in self._type_list:
            instance_client = rospy.ServiceProxy('rosplan_knowledge_base/state/instances', GetInstanceService)
            resp = instance_client(typename)
            item = QTreeWidgetItem(self.instanceView)
            item.setText(0, typename)
            for instancename in resp.instances:
                inst = QTreeWidgetItem(item)
                inst.setText(0, instancename)
            if typename in expanded_list:
                item.setExpanded(True)

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
    called when the plan button is clicked; sends a planning request
    """
    def _handle_plan_clicked(self, checked):
        self._status_list.clear()
        self._plan_pub.publish('plan')

    """
    called when the plan button is clicked; sends a planning request
    """
    def _handle_pause_clicked(self, checked):
        self._status_list.clear()
        self._plan_pub.publish('pause')

    """
    called when the plan button is clicked; sends a planning request
    """
    def _handle_cancel_clicked(self, checked):
        self._status_list.clear()
        self._plan_pub.publish('cancel')

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

    """
    callback for system_state
    """
    def system_status_callback(self, data):
        self.statusLabel.setText(data.data)

    """
    handle changing selected goal or fact predicate name
    """
    def _handle_predicate_name_change(self, predName, combo):
        combo.clear()
        rospy.wait_for_service('rosplan_knowledge_base/state/instances')
        parameters = []
        for param_type in self._predicate_param_type_list[predName]:
            try:
                predicates_client = rospy.ServiceProxy('rosplan_knowledge_base/state/instances', GetInstanceService)
                resp = predicates_client(param_type)
                parameters.append(resp.instances)
            except rospy.ServiceException as e:
                rospy.logerr(f'Service call failed: {e}')
        for element in product(*parameters):
            pred = ''
            for single_element in element:
                pred += f'{single_element} '
            combo.addItem(pred[:-1])

    def _handle_goal_name_changed(self, index):
        self._handle_predicate_name_change(self.goalNameComboBox.itemText(index), self.goalComboBox)

    def _handle_fact_name_changed(self, index):
        self._handle_predicate_name_change(self.factNameComboBox.itemText(index), self.factComboBox)

    """
    called when the add goal/fact button is clicked
    """
    def _handle_add_button_clicked(self, updateType, predName, combo):
        rospy.wait_for_service('rosplan_knowledge_base/update')
        try:
            update_client = rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
            knowledge = KnowledgeItem()
            knowledge.knowledge_type = KnowledgeItem.FACT
            knowledge.attribute_name = predName
            index = 0
            for param in str.split(combo.currentText()):
                pair = KeyValue()
                pair.key = (self._predicate_param_label_list[knowledge.attribute_name])[index]
                index = index + 1
                pair.value = param
                knowledge.values.append(pair)
            resp = update_client(updateType, knowledge)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')

    def _handle_add_goal_clicked(self, data):
        self._handle_add_button_clicked(KnowledgeUpdateServiceRequest.ADD_GOAL, self.goalNameComboBox.currentText(), self.goalComboBox)
        self.refresh_model()

    def _handle_add_fact_clicked(self, data):
        self._handle_add_button_clicked(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, self.factNameComboBox.currentText(), self.factComboBox)
        self.refresh_model()

    """
    called when the remove goal button is clicked
    """
    def _handle_remove_button_clicked(self, updateType, removeNameList, removeMsgList):
        rospy.wait_for_service('rosplan_knowledge_base/update')
        for item in removeNameList:
            try:
                update_client = rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
                resp = update_client(updateType, removeMsgList[item.text()])
            except rospy.ServiceException as e:
                rospy.logerr(f'Service call failed: {e}')
        self.refresh_model()

    def _handle_remove_goal_clicked(self, checked):
        self._handle_remove_button_clicked(KnowledgeUpdateServiceRequest.REMOVE_GOAL, self.goalView.selectedItems(), self._goal_list)
        self.refresh_model()

    def _handle_remove_fact_clicked(self, checked):
        self._handle_remove_button_clicked(KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE, self.modelView.selectedItems(), self._fact_list)
        self.refresh_model()

    """
    called when the add isntance button is clicked
    """
    def _handle_add_instance_clicked(self, checked):
        if self.instanceNameEdit.text() == '':
            return
        rospy.wait_for_service('rosplan_knowledge_base/update')
        try:
            update_client = rospy.ServiceProxy('rosplan_knowledge_base/update', KnowledgeUpdateService)
            knowledge = KnowledgeItem()
            knowledge.knowledge_type = KnowledgeItem.INSTANCE
            knowledge.instance_type = self.typeComboBox.currentText()
            knowledge.instance_name = self.instanceNameEdit.text()
            resp = update_client(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE, knowledge)
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')
        self.refresh_model()

    """
    Qt methods
    """ 
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
