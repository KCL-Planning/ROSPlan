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

from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
if QT_BINDING_VERSION.startswith('4'):
    from python_qt_binding.QtGui import QHeaderView, QIcon, QTreeWidgetItem, QListWidgetItem, QComboBox, QWidget
else:
    from python_qt_binding.QtWidgets import QHeaderView, QTreeWidgetItem, QListWidgetItem, QComboBox, QWidget
    from python_qt_binding.QtGui import QIcon

class ActionDispatchWidget(QWidget):

    # operator view
    _operator_list = []
    _parameter_type_list = {}
    _parameter_label_list = {}

    def __init__(self, plugin=None):
        super(ActionDispatchWidget, self).__init__()

        # Create QWidget
        ui_file = os.path.join(rospkg.RosPack().get_path('rosplan_rqt'), 'resource', 'action_dispatcher.ui')
        loadUi(ui_file, self)
        self.setObjectName('ROSPlanActionDispatcherUI')

        # populate combo box
        rospy.wait_for_service('/kcl_rosplan/get_domain_operators')
        try:
            operator_client = rospy.ServiceProxy('/kcl_rosplan/get_domain_operators', GetDomainOperatorService)
            resp = operator_client()
            for op in resp.operators:
                self.operatorNameComboBox.addItem(op.name)
                param_list = []
                label_list = []
                for param in op.typed_parameters:
                    param_list.append(param.value)
                    label_list.append(param.key)
                self._parameter_type_list[op.name] = param_list
                self._parameter_label_list[op.name] = label_list
        except rospy.ServiceException as e:
            rospy.logerr(f'Service call failed: {e}')
        self._handle_operator_name_changed(0)

        # connect components
        self.dispatchButton.clicked[bool].connect(self._handle_dispatch_clicked)
        self.operatorNameComboBox.currentIndexChanged[int].connect(self._handle_operator_name_changed)

        self._plugin = plugin

        rospy.Subscriber("/kcl_rosplan/action_feedback", ActionFeedback, self.action_feedback_callback)

    """
    called when the dispatch button is clicked; sends action
    """
    def _handle_dispatch_clicked(self, checked):
        pub = rospy.Publisher('/kcl_rosplan/action_dispatch', ActionDispatch, queue_size=10)
        self.statusLabel.setText("")
        msg = ActionDispatch()
        msg.name = self.operatorNameComboBox.currentText()
        root = self.operatorView.invisibleRootItem()
        child_count = root.childCount()
        for i in range(child_count):
            item = root.child(i)
            cmb = self.operatorView.itemWidget(item, 2)
            kv = KeyValue()
            kv.key = item.text(0)
            kv.value = cmb.currentText()
            msg.parameters.append(kv)
        pub.publish(msg)

    """
    callback for action_feedback
    """
    def action_feedback_callback(self, data):
        self.statusLabel.setText(data.status)

    """
    handle changing selected operator name
    """
    def _handle_operator_name_change(self, operatorName, combo):
        self.operatorView.clear()
        root = self.operatorView.invisibleRootItem()
        for i in range(len(self._parameter_label_list[operatorName])):
            # fetch types for combo box
            cmb = QComboBox()
            instance_client = rospy.ServiceProxy('/kcl_rosplan/get_current_instances', GetInstanceService)
            resp = instance_client(self._parameter_type_list[operatorName][i])
            for instancename in resp.instances:
                cmb.addItem(instancename,instancename)
            # create row
            item = QTreeWidgetItem(self.operatorView)
            item.setText(0, self._parameter_label_list[operatorName][i])
            item.setText(1, self._parameter_type_list[operatorName][i])
            self.operatorView.setItemWidget(item, 2, cmb)

    def _handle_operator_name_changed(self, index):
        self._handle_operator_name_change(self.operatorNameComboBox.itemText(index), self.operatorNameComboBox)

    """
    Qt methods
    """ 
    def start(self):
        pass

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
