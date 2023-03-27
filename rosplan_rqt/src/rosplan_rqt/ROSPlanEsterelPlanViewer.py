#!/usr/bin/env python3

import os
import rospy
import rospkg
from std_msgs.msg import String

# reused from ros smach
from rosplan_rqt.xdot_qt import DotWidget

# common imports that work for both versions PyQt4 and PyQt5
from python_qt_binding import loadUi, QT_BINDING_VERSION

# check user Qt version and import libraries accordingly
if QT_BINDING_VERSION.startswith('4'):
    from PyQt4.QtGui import QWidget, QFileDialog
elif QT_BINDING_VERSION.startswith('5'):
    from PyQt5.QtWidgets import QWidget, QFileDialog
else:
    raise ValueError('Unsupported Qt version, supported versions: PyQt4, PyQt5')


class EsterelPlanViewerWidget(QWidget):

    def __init__(self, plugin=None):
        super(EsterelPlanViewerWidget, self).__init__()

        # Create QWidget
        ui_file = os.path.join(rospkg.RosPack().get_path('rosplan_rqt'), 'resource', 'esterel_plan_viewer.ui')
        loadUi(ui_file, self, {'DotWidget':DotWidget})
        self.setObjectName('ROSPlanEsterelPlanViewer')

        self.refreshButton.clicked[bool].connect(self._handle_refresh_clicked)
        self.saveButton.clicked[bool].connect(self._handle_save_button_clicked)

        self._sub = rospy.Subscriber('/rosplan_plan_dispatcher/plan_graph', String, self.planReceivedCallback)

        # flag used to zoom out to fit graph the first time it's received
        self.first_time_graph_received = True
        # to store the graph msg received in callback, later on is used to save the graph if needed
        self.graph = None
        # inform user that no graph has been received by drawing a single node in the rqt
        self.gen_single_node('no plan received')

    def gen_single_node(self, node_text):
        '''
        input: the node content (text)
        return dot code corresponding to a graph of 1 node
        '''
        # generate dot code (of a single node) from received text
        graph = 'digraph plan {0[ label="' + node_text + '",style=filled,fillcolor=white,fontcolor=black];}'
        # render single node graph
        self.xdot_widget.set_dotcode(graph)
        # zoom the single node to be clearly visible
        self.xdot_widget.zoom_image(5.0, center=True)

    def planReceivedCallback(self, msg):
        '''
        updating plan view
        '''
        # save graph in member variable in case user clicks save button later
        self.graph = msg.data
        # render graph using DotWidget class
        rospy.loginfo('Rendering graph started...')
        # inform the user his graph is being rendered
        if self.first_time_graph_received:
            self.gen_single_node('Plan received! rendering...')
        # start rendering graph, might take a while depending on the graph size
        self.xdot_widget.set_dotcode(msg.data)
        if self.first_time_graph_received:
            # zoom out until graph fits in screen
            self.xdot_widget.zoom_to_fit()
            # only zoom to fit for the first graph
            self.first_time_graph_received = False
        self.xdot_widget.update()
        rospy.loginfo('Rendering graph ended !')

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
                         ' clicking "Update subscriber" button and make sure graph is published at least one time'\
                         , self.topicText.text())

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
