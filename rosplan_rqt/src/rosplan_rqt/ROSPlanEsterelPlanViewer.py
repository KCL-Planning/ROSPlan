#!/usr/bin/env python

import os
import rospy
import rospkg
import sys

import pydot

from itertools import product
from string import join, split

from std_msgs.msg import *
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import *
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import Qt, QUrl, pyqtSignal, QRectF, QByteArray
from python_qt_binding.QtSvg import QGraphicsSvgItem
from python_qt_binding.QtWebKit import QGraphicsWebView

if QT_BINDING_VERSION.startswith('4'):
    from python_qt_binding.QtGui import  QWidget, QGraphicsScene, QGraphicsItem, QGraphicsEllipseItem
    from python_qt_binding.QtSvg import  QGraphicsSvgItem, QSvgRenderer
else:
    from python_qt_binding.QtWidgets import QWidget

class EsterelPlanViewerWidget(QWidget):

    # plan view
    _scene = QGraphicsScene()
    _webview = QGraphicsWebView()
    _svg = QGraphicsSvgItem()
    _renderer = QSvgRenderer()

    def __init__(self, plugin=None):
        super(EsterelPlanViewerWidget, self).__init__()

        # Create QWidget
        ui_file = os.path.join(rospkg.RosPack().get_path('rosplan_rqt'), 'resource', 'esterel_plan_viewer.ui')
        loadUi(ui_file, self)
        self.setObjectName('ROSPlanEsterelPlanViewer')

        self.graphicsView.setScene(self._scene)
        self._scene.addItem(self._svg)

        self.refreshButton.clicked[bool].connect(self._handle_refresh_clicked)

        self._sub = rospy.Subscriber("/plan_graph", String, self.plan_received)

        self._plugin = plugin

    """
    updating plan view
    """
    def plan_received(self, msg):
        graph = pydot.graph_from_dot_data(msg.data)
        svg_string = graph.create_svg()
        self._renderer.load(QByteArray(svg_string))
        self._svg.setSharedRenderer(self._renderer)

    """
    called when the refresh button is clicked
    """
    def _handle_refresh_clicked(self, checked):
        self._sub.unregister()
        self._sub = rospy.Subscriber(self.topicText.text(), String, self.plan_received)

    """
    Qt methods
    """ 
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
