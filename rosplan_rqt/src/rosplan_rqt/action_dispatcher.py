from rqt_gui_py.plugin import Plugin

from .ROSPlanActionDispatcher import ActionDispatchWidget

class ROSPlanActionDispatcher(Plugin):

    def __init__(self, context):
        super(ROSPlanActionDispatcher, self).__init__(context)
        self.setObjectName('ActionDispatcher')

        self._widget = ActionDispatchWidget(self)

        self._widget.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
