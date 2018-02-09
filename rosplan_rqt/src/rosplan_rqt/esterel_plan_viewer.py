from rqt_gui_py.plugin import Plugin

from .ROSPlanEsterelPlanViewer import EsterelPlanViewerWidget

class ROSPlanEsterelPlanViewer(Plugin):

    def __init__(self, context):
        super(ROSPlanEsterelPlanViewer, self).__init__(context)
        self.setObjectName('EsterelPlanViewer')

        self._widget = EsterelPlanViewerWidget(self)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
