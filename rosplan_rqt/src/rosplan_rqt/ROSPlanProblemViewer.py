#!/usr/bin/env python

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

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, QUrl, Signal, Slot
from python_qt_binding.QtGui import *

class ProblemViewerWidget(QWidget):

    _problem_text = ""

    def __init__(self, plugin=None):
        super(ProblemViewerWidget, self).__init__()

        # Create QWidget
        ui_file = os.path.join(rospkg.RosPack().get_path('rosplan_rqt'), 'resource', 'problem_viewer.ui')
        loadUi(ui_file, self)
        self.setObjectName('ROSPlanProblemViewer')

        self._plugin = plugin
        rospy.Subscriber("/kcl_rosplan/problem", String, self.refresh_problem)

        # init and start update timers
        self._timer_set_problem = QTimer(self)
        self._timer_set_problem.timeout.connect(self.set_problem)

        self.start()

    def start(self):
        self._timer_set_problem.start(1000)

    """
    updating problem view
    """
    def refresh_problem(self, msg):
        self._problem_text = msg.data

    def set_problem(self):
        self.textEdit.setPlainText(self._problem_text)

    """
    Qt methods
    """ 
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
