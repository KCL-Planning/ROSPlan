#!/usr/bin/env python

import os
import pydot

import rospy
import rospkg
from std_msgs.msg import String
# for resizing graph we use functionality from rqt_graph
from rqt_graph.interactive_graphics_view import InteractiveGraphicsView

# common imports that work for both versions PyQt4 and PyQt5
from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtCore import QByteArray
from python_qt_binding.QtSvg import QGraphicsSvgItem

# check user Qt version and import libraries accordingly
if QT_BINDING_VERSION.startswith('4'):
    from python_qt_binding.QtGui import  QWidget, QGraphicsScene
    from python_qt_binding.QtSvg import  QSvgRenderer
    from python_qt_binding.QtWebKit import QGraphicsWebView
elif QT_BINDING_VERSION.startswith('5'):
    from PyQt5.QtWidgets import QWidget
    from PyQt5.QtWebKitWidgets import QGraphicsWebView
    from PyQt5.QtWidgets import QGraphicsScene
    from PyQt5.QtSvg import QSvgRenderer
    from PyQt5.QtWidgets import QFileDialog
else:
    rospy.logerr('Unsupported Qt version, (supported are : PyQt4, PyQt5)')


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
        loadUi(ui_file, self, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self.setObjectName('ROSPlanEsterelPlanViewer')

        self.graphicsView.setScene(self._scene)
        self._scene.addItem(self._svg)

        self.refreshButton.clicked[bool].connect(self._handle_refresh_clicked)
        self.saveButton.clicked[bool].connect(self._handle_save_button_clicked)

        self._sub = rospy.Subscriber('/rosplan_plan_dispatcher/plan_graph', String, self.planReceivedCallback)
        self._plugin = plugin

        # to store the graph msg received in callback, later on is used to save the graph if needed
        self.graph = None

    def planReceivedCallback(self, msg):
        '''
        updating plan view
        '''
        # save graph in member variable in case user clicks save button later
        self.graph = msg.data
        # render graph
        temp_buffer_graph = pydot.graph_from_dot_data(self.graph)
        svg_string = temp_buffer_graph.create_svg()
        self._renderer.load(QByteArray(svg_string))
        self._svg.setSharedRenderer(self._renderer)

    def _handle_refresh_clicked(self, checked):
        '''
        called when the refresh button is clicked
        '''
        self._sub.unregister()
        self._sub = rospy.Subscriber(self.topicText.text(), String, self.planReceivedCallback)

    def save_graph(self, full_path):
        '''
        check if last graph msg received is valid (non empty), then save in file.dot
        '''
        if self.graph:
            dot_file = open(full_path,'w')
            dot_file.write(self.graph)
            dot_file.close()
            rospy.loginfo('graph saved succesfully in %s', full_path)
        else:
            # if self.graph is None it will fall in this case
            rospy.logerr('Could not save Graph: is empty, currently subscribing to: %s, try' +\
                         ' clicking "Update subscriber" button and make sure graph is published at least one time', self.topicText.text())

    def _handle_save_button_clicked(self, checked):
        '''
        called when the save button is clicked
        '''
        rospy.loginfo('Saving esterel plan to dot file')
        fileName = QFileDialog.getSaveFileName(self, 'Save Esterel plan to dot file','','Graph xdot Files (*.dot)')
        if fileName[0] == '':
            rospy.loginfo("User has cancelled saving process")
        else:
            # add .dot at the end of the filename
            full_dot_path = fileName[0]
            if not '.dot' in full_dot_path:
                full_dot_path += '.dot'
            rospy.loginfo("path to save dot file: %s", full_dot_path)
            self.save_graph(full_dot_path)

    # Qt methods
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
