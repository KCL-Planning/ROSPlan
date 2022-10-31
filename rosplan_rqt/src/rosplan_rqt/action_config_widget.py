import os
import rospkg
import yaml
from os.path import expanduser

from python_qt_binding import loadUi, QT_BINDING_VERSION
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel, QPixmap
from python_qt_binding.QtWidgets import QMainWindow, QAbstractItemView

if QT_BINDING_VERSION.startswith('4'):
    from PyQt4.QtGui import QFileDialog
elif QT_BINDING_VERSION.startswith('5'):
    from PyQt5.QtWidgets import QFileDialog
else:
    raise ValueError('Unsupported Qt version, supported versions: PyQt4, PyQt5')


#Popup window for actionlib param settings
class popupActionlibWidget(QMainWindow):
    def __init__(self):
        super(popupActionlibWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rosplan_rqt'), 'resource', 'ActionConfig_popup_actionlib.ui')
        loadUi(ui_file, self)

        self.param_values_data = []

        #---------------- signals ---------------------------------------------
        self.pushButton_popup_actionlib_save.clicked[bool].connect(self._handle_popup_save)
        self.pushButton_popup_actionlib_cancel.clicked[bool].connect(self._handle_popup_cancel)

    #----------------------slots-----------------------------------------------
    # save input callback function
    def _handle_popup_save(self):
        self.close()
        values = yaml.load('[' + self.lineEdit_popup_actionlib_values.text() + ']')
        actionlib_msgType = self.lineEdit_popup_actionlib_msgType.text()
        actionlib_topic = self.lineEdit_popup_actionlib_topic.text()
        actionlib_goal = yaml.load(self.textEdit_popup_actionlib_goal.toPlainText())
        fill = {'values': values,
                'actionlib_msg_type': actionlib_msgType,
                'actionlib_topic': actionlib_topic,
                'actionlib_goal': actionlib_goal
                 }

        # ToDo: Modify the quick and dirty solution
        # Check the param values
        new_param_values_data = True
        for i in range(len(self.param_values_data)):
            # If they are the same replace the entry
            if self.param_values_data[i]['values'] == fill['values']:
                new_param_values_data = False
                self.param_values_data[i] = fill
                break
        # If the param values were not found in another entry of the list, extend the list
        if new_param_values_data:
            self.param_values_data.append(fill)
        self.clearInput()

    #delete all previous entries in window
    def clearInput(self):
        self.lineEdit_popup_actionlib_values.clear()
        self.lineEdit_popup_actionlib_msgType.clear()
        self.lineEdit_popup_actionlib_topic.clear()
        self.textEdit_popup_actionlib_goal.clear()

    def _handle_popup_cancel(self):
        self.close()

#Popup window for service param settings
class popupServiceWidget(QMainWindow):
    def __init__(self):
        super(popupServiceWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rosplan_rqt'), 'resource', 'ActionConfig_popup_service.ui')
        loadUi(ui_file, self)

        self.param_values_data = []

        #------------------------------- Signal --------------------------------------------
        self.pushButton_popup_service_save.clicked[bool].connect(self._handle_popup_save)
        self.pushButton_popup_service_cancel.clicked[bool].connect(self._handle_popup_cancel)

    # save input callback function
    def _handle_popup_save(self):
        self.close()
        values = yaml.load('[' + self.lineEdit_popup_service_values.text() + ']')
        service_type = self.lineEdit_popup_service_type.text()
        service = self.lineEdit_popup_service.text()
        service_request = yaml.load(self.textEdit_popup_service_request.toPlainText())
        service_response = yaml.load(self.textEdit_popup_service_response.toPlainText())
        fill = {'values': values,
                'service_type': service_type,
                'service': service,
                'service_request': service_request,
                'service_response': service_response
               }

        # ToDo: Modify the quick and dirty solution
        # Check the param values
        new_param_values_data = True
        for i in range(len(self.param_values_data)):
            # If they are the same replace the entry
            if self.param_values_data[i]['values'] == fill['values']:
                new_param_values_data = False
                self.param_values_data[i] = fill
                break
        # If the param values were not found in another entry of the list, extend the list
        if new_param_values_data:
            self.param_values_data.append(fill)
        self.clearInput()

    #delete all previous entries in window
    def clearInput(self):
        self.lineEdit_popup_service_values.clear()
        self.lineEdit_popup_service_type.clear()
        self.lineEdit_popup_service.clear()
        self.textEdit_popup_service_request.clear()
        self.textEdit_popup_service_response.clear()

    def _handle_popup_cancel(self):
        self.close()

#Main GUI window
class ActionConfigWidget(QMainWindow):
    def __init__(self):
        super(ActionConfigWidget, self).__init__()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rosplan_rqt'), 'resource', 'ActionConfig.ui')
        loadUi(ui_file, self)

        # -------------------------- variable init ---------------------------------
        self.internal_data = [] #main data
        self.update_signal = False #update data checker
        self.popup_actionlib = popupActionlibWidget() #popup actionlib window
        self.popup_service = popupServiceWidget() #popup service window

        # ---------------------------- ui setup ------------------------------------
        self.setObjectName('ActionConfigUi')
        self.pushButton_add_state.setDisabled(True)
        self.stackedWidget_newAction_addState.setCurrentIndex(1)
        self.treeView_overview.setHeaderHidden(True)
        self.logo = QPixmap(':/logo/igmr_logo.png')
        self.label_logo.setPixmap(self.logo)
        self.resize(self.logo.width(),self.logo.height())

        # ------------------------ overview model -----------------------------------
        self.model_overview = QStandardItemModel()
        self.treeView_overview.setModel(self.model_overview)
        self.treeView_overview.model().setHorizontalHeaderLabels(['Action', 'Interface Type', 'id'])
        self.treeView_overview.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.treeView_overview.setColumnWidth(0,50)
        self.treeView_overview.setColumnWidth(1,350)
        self.treeView_overview.setColumnWidth(2,10)

        # ---------------------  TableView models  -----------------------------------

        # tableview -> newAction_actionlib
        self.tableView_model_newAction_actionlib = QStandardItemModel()
        self.tableView_newAction_actionlib_paramValues.setModel(self.tableView_model_newAction_actionlib)
        self.tableView_newAction_actionlib_paramValues.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableView_newAction_actionlib_paramValues.verticalHeader().setVisible(False)
        self.tableView_newAction_actionlib_paramValues.horizontalHeader().setVisible(False)
        self.tableView_newAction_actionlib_paramValues.horizontalHeader().setStretchLastSection(True)

        # tableview -> newAction_service
        self.tableView_model_newAction_service = QStandardItemModel()
        self.tableView_newAction_service_paramValues.setModel(self.tableView_model_newAction_service)
        self.tableView_newAction_service_paramValues.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableView_newAction_service_paramValues.verticalHeader().setVisible(False)
        self.tableView_newAction_service_paramValues.horizontalHeader().setVisible(False)
        self.tableView_newAction_service_paramValues.horizontalHeader().setStretchLastSection(True)

        # tableview -> addState_actionlib
        self.tableView_model_addState_actionlib = QStandardItemModel()
        self.tableView_addState_actionlib_paramValues.setModel(self.tableView_model_addState_actionlib)
        self.tableView_addState_actionlib_paramValues.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableView_addState_actionlib_paramValues.verticalHeader().setVisible(False)
        self.tableView_addState_actionlib_paramValues.horizontalHeader().setVisible(False)
        self.tableView_addState_actionlib_paramValues.horizontalHeader().setStretchLastSection(True)

        #tableview -> addState_service
        self.tableView_model_addState_service = QStandardItemModel()
        self.tableView_addState_service_paramValues.setModel(self.tableView_model_addState_service)
        self.tableView_addState_service_paramValues.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.tableView_addState_service_paramValues.verticalHeader().setVisible(False)
        self.tableView_addState_service_paramValues.horizontalHeader().setVisible(False)
        self.tableView_addState_service_paramValues.horizontalHeader().setStretchLastSection(True)

        #----------------------- signals (events) ------------------------------------

        self.pushButton_new_action.clicked[bool].connect(self._handle_new_action_clicked)
        self.pushButton_add_state.clicked[bool].connect(self._handle_add_state_clicked)
        self.radioButton_newAction_fsm.clicked[bool].connect(self._handle_new_action_setting_fsm)
        self.radioButton_newAction_actionlib.clicked[bool].connect(self._handle_new_action_setting_actionlib)
        self.radioButton_newAction_service.clicked[bool].connect(self._handle_new_action_setting_service)
        self.radioButton_addState_fsm.clicked[bool].connect(self._handle_add_state_setting_fsm)
        self.radioButton_addState_actionlib.clicked[bool].connect(self._handle_add_state_setting_actionlib)
        self.radioButton_addState_service.clicked[bool].connect(self._handle_add_state_setting_service)
        self.pushButton_newAction_save.clicked[bool].connect(self._handle_new_action_save)
        self.pushButton_newAction_cancel.clicked[bool].connect(self._handle_new_action_cancel)
        self.pushButton_addState_save.clicked[bool].connect(self._handle_add_state_save)
        self.pushButton_addState_cancel.clicked[bool].connect(self._handle_add_state_cancel)
        self.pushButton_overview_delete.clicked[bool].connect(self._handle_delete_action_overview)
        self.treeView_overview.selectionModel().selectionChanged.connect(self._handle_addState_pushButton_clickable)
        self.treeView_overview.doubleClicked.connect(self._handle_overview_doubleClicked)
        self.pushButton_newAction_actionlib_paramValues_add.clicked[bool].connect(self._handle_newAction_actionlib_popup)
        self.pushButton_newAction_service_paramValues_add.clicked[bool].connect(self._handle_newAction_service_popup)
        self.pushButton_addState_actionlib_addParam.clicked[bool].connect(self._handle_addState_actionlib_popup)
        self.pushButton_addState_service_addParam.clicked[bool].connect(self._handle_addState_service_popup)
        self.popup_actionlib.pushButton_popup_actionlib_save.clicked[bool].connect(self._handle_popup_save)
        self.popup_service.pushButton_popup_service_save.clicked[bool].connect(self._handle_popup_save)
        self.tableView_newAction_actionlib_paramValues.doubleClicked.connect(self._handle_tableView_actionlib_doubleClicked)
        self.tableView_newAction_service_paramValues.doubleClicked.connect(self._handle_tableView_service_doubleClicked)
        self.tableView_addState_actionlib_paramValues.doubleClicked.connect(self._handle_tableView_actionlib_doubleClicked)
        self.tableView_addState_service_paramValues.doubleClicked.connect(self._handle_tableView_service_doubleClicked)
        self.pushButton_newAction_actionlib_paramValues_delete.clicked[bool].connect(self._handle_tableView_delete)
        self.pushButton_newAction_service_paramValues_delete.clicked[bool].connect(self._handle_tableView_delete)
        self.pushButton_addState_actionlib_deleteParam.clicked[bool].connect(self._handle_tableView_delete)
        self.pushButton_addState_service_deleteParam.clicked[bool].connect(self._handle_tableView_delete)
        self.pushButton_import.clicked[bool].connect(self._handle_import_file)
        self.pushButton_export.clicked[bool].connect(self._handle_export_file)
        self.pushButton_print_fsm.clicked[bool].connect(self._handle_print_FSM)

    # -----------------------------------------------------------------------------------
    # ------------ Handles: New, Save, Cancel Save --------------------------------------
    # -----------------------------------------------------------------------------------

    #Display new action window
    def _handle_new_action_clicked(self):
        self.clear_new_action_input() # clear all previous inputs
        self.update_signal = False
        self.stackedWidget_newAction_addState.setCurrentIndex(2)
        self.stackedWidget_newAction_setting.setCurrentIndex(2)

    #Display add state window
    def _handle_add_state_clicked(self):
        self.clear_add_state_input() # clear all previous inputs
        self.update_signal = False
        self.stackedWidget_newAction_addState.setCurrentIndex(0)
        self.stackedWidget_addState_setting.setCurrentIndex(2)

    #Display fsm setting (i.e no setting) in new action window
    def _handle_new_action_setting_fsm(self):
        if self.radioButton_newAction_fsm.isChecked():
            # Unset the other radioButtons
            self.radioButton_newAction_actionlib.setChecked(False)
            self.radioButton_newAction_service.setChecked(False)
            # Set the correct widget
            self.stackedWidget_newAction_setting.setCurrentIndex(2)

    #Display actionlib setting in new action window
    def _handle_new_action_setting_actionlib(self):
        if self.radioButton_newAction_actionlib.isChecked():
            # Unset the other radioButtons
            self.radioButton_newAction_fsm.setChecked(False)
            self.radioButton_newAction_service.setChecked(False)
            # Set the correct widget
            self.stackedWidget_newAction_setting.setCurrentIndex(0)

    # Display service setting in new action window
    def _handle_new_action_setting_service(self):
        if self.radioButton_newAction_service.isChecked():
            # Unset the other radioButtons
            self.radioButton_newAction_fsm.setChecked(False)
            self.radioButton_newAction_actionlib.setChecked(False)
            # Set the correct widget
            self.stackedWidget_newAction_setting.setCurrentIndex(1)

    # Display fsm setting (i.e no setting) in add state window
    def _handle_add_state_setting_fsm(self):
        if self.radioButton_addState_fsm.isChecked():
            # Unset the other radioButtons
            self.radioButton_addState_actionlib.setChecked(False)
            self.radioButton_addState_service.setChecked(False)
            # Set the correct widget
            self.stackedWidget_addState_setting.setCurrentIndex(2)

    # Display actionlib setting in add state window
    def _handle_add_state_setting_actionlib(self):
        if self.radioButton_addState_actionlib.isChecked():
            # Unset the other radioButtons
            self.radioButton_addState_fsm.setChecked(False)
            self.radioButton_addState_service.setChecked(False)
            # Set the correct widget
            self.stackedWidget_addState_setting.setCurrentIndex(0)

    # Display service setting in add state window
    def _handle_add_state_setting_service(self):
        if self.radioButton_addState_service.isChecked():
            # Unset the other radioButtons
            self.radioButton_addState_fsm.setChecked(False)
            self.radioButton_addState_actionlib.setChecked(False)
            # Set the correct widget
            self.stackedWidget_addState_setting.setCurrentIndex(1)

    # Save action inputs
    def _handle_new_action_save(self):
        # update action entry
        if self.update_signal:
            self.update_internal_data_from_gui_data(origin="add_action")
            self.set_gui_data_from_internal_data(origin="add_action")

            # Get the rows of the parents of the current index
            rows, main_parent = self.get_rows_to_main_parent_index(self.get_current_index())

            # Reset the treeView overview
            self.reset_treeView_overview()

            # Reset the current index
            index = main_parent
            for row in rows:
                index = index.child(row, 0)
            self.treeView_overview.setCurrentIndex(index)

        else: #Save new action
            newAction_name_value = self.lineEdit_newAction_name.text()
            # check if action name is already existent
            if not any(d['name'] == newAction_name_value for d in self.internal_data):
                if len(newAction_name_value) == 0:
                    return
                self.label_new_action_name_error.clear()

                self.set_internal_data_from_gui_data(origin="add_action")

                # Reset the treeView overview
                self.reset_treeView_overview()

                # Get the number of children of the root of the tree view model
                nr_children = 0
                while not self.model_overview.invisibleRootItem().child(nr_children) is None:
                    nr_children += 1
                nr_children -= 1

                # Select the latest added child (action)
                self.treeView_overview.setCurrentIndex(self.model_overview.invisibleRootItem().child(nr_children).index())

                # Update the data
                self.update_signal = True
            else:
                self.label_new_action_name_error.setText('Name already used. Please chose another name') # display error message if action name was already used

    # save state input
    def _handle_add_state_save(self):
        if self.update_signal == True: # -> update changed state input
            self.update_internal_data_from_gui_data(origin="add_state")
            self.set_gui_data_from_internal_data(origin="add_state")

            # Get the rows of the parents of the current index
            rows, main_parent = self.get_rows_to_main_parent_index(self.get_current_index())

            # Reset the treeView overview
            self.reset_treeView_overview()

            # Go back and set the index
            index = main_parent
            for row in rows:
                index = index.child(row, 0)
            self.treeView_overview.setCurrentIndex(index)

        else: #-> save new state input
            addState_name_value = self.lineEdit_addState_name.text()

            # -> basic action name not yet used
            if not self.check_state_name_present_in_fsm_states(addState_name_value):
                self.set_internal_data_from_gui_data(origin="add_state")

                # Get the rows of the parents of the current index
                rows, main_parent = self.get_rows_to_main_parent_index(self.get_current_index())

                # Get the number of children (states) that the current index (the fsm) has
                current_index = self.get_current_index()
                current_index_nr_children = 0
                while current_index.child(current_index_nr_children, 0).isValid():
                    current_index_nr_children += 1
                rows.append(current_index_nr_children)

                # Reset the treeView overview
                self.reset_treeView_overview()

                # Go back in the tree to the next location of the original current index
                index = main_parent
                for row in rows:
                    index = index.child(row, 0)
                self.treeView_overview.setCurrentIndex(index)

                # Update the data
                self.update_signal = True

            else: # -> basic action used
                #ToDo
                print "Name already exitent. Please give action a new name"

            self.stackedWidget_newAction_addState.setCurrentIndex(1)

    #close new action window
    def _handle_new_action_cancel(self):
        self.stackedWidget_newAction_addState.setCurrentIndex(1)
        self.update_signal = False
        self.clear_new_action_input()

    #close add state window
    def _handle_add_state_cancel(self):
        self.stackedWidget_newAction_addState.setCurrentIndex(1)
        self.update_signal = False
        self.clear_add_state_input()

    # -----------------------------------------------------------------------------------
    # ------------------ Handles: Actions Overview --------------------------------------
    # -----------------------------------------------------------------------------------

    #make push button add state clickable if selected action is fsm
    def _handle_addState_pushButton_clickable(self):
        #self.stackedWidget_newAction_addState.setCurrentIndex(1)
        currentIndex = self.treeView_overview.selectedIndexes()[2]
        item = self.model_overview.itemFromIndex(currentIndex).text()

        if item == "fsm":
            self.pushButton_add_state.setDisabled(False)
            self.pushButton_add_state.setStyleSheet('color:rgb(0, 84, 149)')
        else:
            self.pushButton_add_state.setDisabled(True)
            self.pushButton_add_state.setStyleSheet('color: rgb(48, 48, 48)')

    #load selected action for update
    def _handle_overview_doubleClicked(self):
        self.update_signal = True
        self.get_internal_data_entry_level()
        rows_parents, main_parent = self.get_rows_to_main_parent_index(self.get_current_index())
        if len(rows_parents) == 0:
            self.set_gui_data_from_internal_data(origin="add_action")
        if len(rows_parents) > 0:
            self.set_gui_data_from_internal_data(origin="add_state")

    #delete selected entry of overview
    def _handle_delete_action_overview(self):
        # Get the rows of the parents of the current index
        current_index = self.get_current_index()
        rows = []
        if current_index is None:
            return False

        while current_index.parent().isValid():
            rows.append(current_index.row())
            current_index = current_index.parent()
        main_parent = current_index
        sibling_above_index = self.get_sibling_above_index()

        # If the internal data could have been removed
        if not self.remove_internal_data_level():
            return False

        # Reset the treeView overview
        self.reset_treeView_overview()

        rows.reverse()
        current_index = main_parent
        for row in rows:
            if current_index.child(row, 0).isValid():
                # Get the index of the element from the row at which the deleted element was
                current_index = current_index.child(row, 0)
            elif current_index.child(row - 1, 0).isValid():
                # Get the index of the element from the row above from that where the deleted element was. This
                # is relevant when the deleted element was the last child from that level.
                current_index = current_index.child(row - 1, 0)
            elif current_index.isValid():
                # Get the parent if the deleted element was the only child
                current_index = current_index
        self.treeView_overview.setCurrentIndex(current_index)

        # Special case on the highest level
        if len(rows) == 0:
            if not sibling_above_index is None:
                # Check if there is an entry on the sibling above index row + 1, so at the row where
                if sibling_above_index.sibling(sibling_above_index.row() + 1, 0).isValid():
                    self.treeView_overview.setCurrentIndex(
                        sibling_above_index.sibling(sibling_above_index.row() + 1, 0))
                else:
                    self.treeView_overview.setCurrentIndex(sibling_above_index)

    # -----------------------------------------------------------------------------------
    # ------------- Handles: Parameters Table View and Pop-ups --------------------------
    # -----------------------------------------------------------------------------------

    # callback function when parameter set in tableview actionlib is double clicked
    def _handle_tableView_actionlib_doubleClicked(self):
        internal_data_entry_level = self.get_internal_data_entry_level()
        param_values = internal_data_entry_level['parameter_values']
        indexes = self.tableView.selectionModel().selectedRows()
        selectedIndex = 0
        for index in sorted(indexes):
            selectedIndex = index.row() #return the row number of selected item on tableview
        refill_list = param_values[selectedIndex]
        values = refill_list.get('values', '')
        if values is None:
            self.popup_actionlib.lineEdit_popup_actionlib_values.setText("")
        else:
            self.popup_actionlib.lineEdit_popup_actionlib_values.setText(yaml.dump(values)[1:-2])
        self.popup_actionlib.lineEdit_popup_actionlib_msgType.setText(refill_list.get('actionlib_msg_type', ''))
        self.popup_actionlib.lineEdit_popup_actionlib_topic.setText(refill_list.get('actionlib_topic', ''))
        actionlib_goal = refill_list.get('actionlib_goal', '')
        if actionlib_goal is None:
            self.popup_actionlib.textEdit_popup_actionlib_goal.setText("")
        else:
            self.popup_actionlib.textEdit_popup_actionlib_goal.setText(yaml.dump(actionlib_goal,
                                                                       default_flow_style=False))
        self.popup_actionlib.show()

    #same as above but for tableview service
    def _handle_tableView_service_doubleClicked(self):
        internal_data_entry_level = self.get_internal_data_entry_level()
        param_values = internal_data_entry_level['parameter_values']
        selectedIndex = 0
        indexes = self.tableView.selectionModel().selectedRows()
        for index in sorted(indexes):
            selectedIndex = index.row()
        refill_list = param_values[selectedIndex]
        values = refill_list.get('values', '')
        if values is None:
            self.popup_service.lineEdit_popup_service_values.setText("")
        else:
            self.popup_service.lineEdit_popup_service_values.setText(yaml.dump(values)[1:-2])
        self.popup_service.lineEdit_popup_service_type.setText(refill_list.get('service_type', ''))
        self.popup_service.lineEdit_popup_service.setText(refill_list.get('service', ''))
        service_request = refill_list.get('service_request', '')
        if service_request is None:
            self.popup_service.textEdit_popup_service_request.setText("")
        else:
            self.popup_service.textEdit_popup_service_request.setText(yaml.dump(service_request,
                                                                   default_flow_style=False))

        service_response = refill_list.get('service_response', '')
        if service_response is None:
            self.popup_service.textEdit_popup_service_response.setText("")
        else:
            self.popup_service.textEdit_popup_service_response.setText(yaml.dump(service_response,
                                                                               default_flow_style=False))
        self.popup_service.show()

    #delete selected item in tableview
    def _handle_tableView_delete(self):
        internal_data_entry_level = self.get_internal_data_entry_level()
        indexes = self.tableView.selectionModel().selectedRows()
        selected_index = 0
        for index in sorted(indexes):
            selected_index = index.row()
        internal_data_entry_level['parameter_values'].pop(selected_index)
        self.tableView.selectionModel().reset()
        self.view_model.removeRows(0, self.view_model.rowCount())
        self.fill_params_tableView(internal_data_entry_level['parameter_values'])

    #show actionlib popup window
    def _handle_newAction_actionlib_popup(self):
        self.popup_actionlib.clearInput()
        self.tableView = self.tableView_newAction_actionlib_paramValues
        self.popup_actionlib.show()

    #show service popup window
    def _handle_newAction_service_popup(self):
        self.popup_service.clearInput()
        self.tableView = self.tableView_newAction_service_paramValues
        self.popup_service.show()

    #show actionlib popup window
    def _handle_addState_actionlib_popup(self):
        self.popup_actionlib.clearInput()
        self.tableView = self.tableView_addState_actionlib_paramValues
        self.popup_actionlib.show()

    #show service popup window
    def _handle_addState_service_popup(self):
        self.popup_service.clearInput()
        self.tableView = self.tableView_addState_service_paramValues
        self.popup_service.show()

    #popup windows save button callback
    def _handle_popup_save(self):
        if self.tableView == self.tableView_newAction_actionlib_paramValues: #new action actionlib tableview
            self.tableView.selectionModel().reset()
            self.tableView_model_newAction_actionlib.removeRows(0, self.tableView_model_newAction_actionlib.rowCount())
            self.view_model = self.tableView_model_newAction_actionlib
            tableView_entry = self.popup_actionlib.param_values_data

        elif self.tableView == self.tableView_newAction_service_paramValues: #new action service tableview
            self.tableView.selectionModel().reset()
            self.tableView_model_newAction_service.removeRows(0, self.tableView_model_newAction_service.rowCount())
            self.view_model = self.tableView_model_newAction_service
            tableView_entry = self.popup_service.param_values_data

        elif self.tableView == self.tableView_addState_actionlib_paramValues: #add state actionlib tableview
            self.tableView.selectionModel().reset()
            self.tableView_model_addState_actionlib.removeRows(0, self.tableView_model_addState_actionlib.rowCount())
            self.view_model = self.tableView_model_addState_actionlib
            tableView_entry = self.popup_actionlib.param_values_data

        elif self.tableView == self.tableView_addState_service_paramValues: #add state service tableview
            self.tableView.selectionModel().reset()
            self.tableView_model_addState_service.removeRows(0, self.tableView_model_addState_service.rowCount())
            self.view_model = self.tableView_model_addState_service
            tableView_entry = self.popup_service.param_values_data

        self.fill_params_tableView(tableView_entry)

    # -----------------------------------------------------------------------------------
    # ----------------------- Handles: Utilities ----------------------------------------
    # -----------------------------------------------------------------------------------

    def _handle_export_file(self):
        data = self.internal_data
        self.strip_empties_from_list(data)
        final_data = {'actions': self.new_data}
        file = QFileDialog.getSaveFileName(self, "yaml file", expanduser("~"), "yaml (*.yaml)")
        file = str(file[0])
        if file.find('.yaml') == -1:
            file = open(file + '.yaml', 'w+')
        else:
            file = open(file, 'w+')
        yaml.safe_dump(final_data, file, encoding='utf-8', allow_unicode=True, default_flow_style=False)

    def _handle_import_file(self):
        config_file = QFileDialog.getOpenFileName(self, "yaml file", expanduser("~"), "yaml (*.yaml)")
        if len(config_file[0]) > 0:
            with open(config_file[0], 'r') as stream:
                self.internal_data = yaml.safe_load(stream)["actions"]

            self.treeView_overview.blockSignals(True)
            self.treeView_overview.selectionModel().reset()
            self.model_overview.removeRows(0, self.model_overview.rowCount())
            self.treeView_overview.blockSignals(False)
            self.fill_model_overview(self.model_overview.invisibleRootItem(), self.internal_data, 0)

    def _handle_print_FSM(self):
        graph = "digraph actions_configs { \n compound=true; \n"
        graph += "subgraph cluster_no_name { \n"

        internal_data_entry_level = self.get_internal_data_entry_level()
        if internal_data_entry_level.has_key('states'):
            # Continue only if a fsm child was selected
            for state in internal_data_entry_level['states']:
                # Create node
                graph += state["name"] + "[label=\"" + state["name"] + "\"]; \n"

                # Create transitions
                if state.has_key("transitions"):
                    for transition in state["transitions"]["succeeded"]:
                        # Only if the to_state tag has an input
                        if len(transition["to_state"]) > 0:
                            graph += state["name"] + "->" + transition["to_state"] + \
                                     "[label=\"succeeded\" ]; \n"
                    for idx, transition in enumerate(state["transitions"]["failed"]):
                        # Only if the to_state tag has an input
                        if len(transition["to_state"]) > 0:
                            graph += state["name"] + "->" + transition["to_state"] + \
                                     "[label=\"failed_" + str(idx) + "\" ]; \n"

        graph += "{rank=source start_state;} \n {rank=sink goal_state;} \n"
        graph += "}\n}\n"

        text_file = open(os.path.expanduser('~/fsm_printed_from_the_gui.dot'), "w")
        text_file.write(graph)
        text_file.close()

        os.system('dot -Tpdf ~/fsm_printed_from_the_gui.dot > ~/fsm_printed_from_the_gui.pdf && '
                  'evince ~/fsm_printed_from_the_gui.pdf')

    # -----------------------------------------------------------------------------------
    # ---- Indexes related to the selected action interface from the overview -----------
    # -----------------------------------------------------------------------------------

    def get_current_index(self):
        if len(self.treeView_overview.selectedIndexes()) > 0:
            return self.treeView_overview.selectedIndexes()[0]
        else:
            return None

    def get_sibling_above_index(self):
        current_index = self.treeView_overview.selectedIndexes()[0]
        if current_index.row() > 0:
           # Return the sibling from the above row
            return current_index.sibling(current_index.row() - 1, 0)
        else:
            return None

    def get_rows_to_main_parent_index(self, index):
        rows = []
        if index is None:
            return rows

        while index.parent().isValid():
            rows.append(index.row())
            index = index.parent()
        rows.reverse()

        main_parent_index = index
        return rows, main_parent_index

    # -----------------------------------------------------------------------------------
    # ---------------- Manipulate internal data on the entry level ----------------------
    # -----------------------------------------------------------------------------------

    def get_internal_data_entry_level(self):
        current_index = self.get_current_index()
        rows_parents, main_parent = self.get_rows_to_main_parent_index(current_index)
        internal_data_entry_level = self.internal_data[main_parent.row()]
        for row in rows_parents:
            internal_data_entry_level = internal_data_entry_level['states'][row]
        return internal_data_entry_level

    def set_internal_data_entry_level(self, input_data):
        current_index = self.get_current_index()
        rows_parents, main_parent = self.get_rows_to_main_parent_index(current_index)
        internal_data_entry_level = self.internal_data[main_parent.row()]
        for row in rows_parents:
            internal_data_entry_level = internal_data_entry_level['states'][row]
        internal_data_entry_level = input_data

    def remove_internal_data_level(self):
        current_index = self.get_current_index()
        rows_parents, main_parent = self.get_rows_to_main_parent_index(current_index)
        internal_data_level = self.internal_data
        if len(rows_parents) == 0:
            # Highest level
            if len(internal_data_level) > 0:
                # Delete data if there is at least one action interface
                internal_data_level.pop(main_parent.row())
                return True
            else:
                return False
        else:
            # In the fsms
            internal_data_level = internal_data_level[main_parent.row()]
            for row in rows_parents[0:-1]:
                internal_data_level = internal_data_level['states'][row]
            internal_data_level['states'].pop(rows_parents[-1])
            return True

    # -----------------------------------------------------------------------------------
    # ---------- Mappings of the entered data and the internal data ---------------------
    # -----------------------------------------------------------------------------------

    # Set internal data from GUI data
    def set_internal_data_from_gui_data(self, origin):

        if origin == "add_action":
            # Actions on the highest level
            fill = {}
            newAction_name_value = self.lineEdit_newAction_name.text()
            if self.radioButton_newAction_fsm.isChecked() == True:  # action is fsm
                interfaceType_value = 'fsm'
                fill = {'name': newAction_name_value,
                        'interface_type': interfaceType_value,
                        'states': []
                        }
                self.internal_data.append(fill)


            if self.radioButton_newAction_actionlib.isChecked() == True:  # action is actionlib
                interfaceType_value = 'actionlib'
                default_msgType_value = self.lineEdit_newAction_default_actionlib_msgType.text()
                default_topic_value = self.lineEdit_newAction_default_actionlib_topic.text()
                default_goal_value = yaml.load(self.textEdit_newAction_default_actionlib_goal.toPlainText())
                pddl_param_value = yaml.load('[' + self.lineEdit_newAction_actionlib_pddlParam.text() + ']')
                fill = {'name': newAction_name_value,
                        'interface_type': interfaceType_value,
                        'default_actionlib_msg_type': default_msgType_value,
                        'default_actionlib_topic': default_topic_value,
                        'default_actionlib_goal': default_goal_value,
                        'pddl_parameters': pddl_param_value,
                        'parameter_values': self.popup_actionlib.param_values_data
                        }
                # fill main data with new action entry
                self.internal_data.append(fill)
                self.popup_actionlib.param_values_data = []  # parameter entry from popup actionlib window

            if self.radioButton_newAction_service.isChecked() == True:  # action is service
                interfaceType_value = 'service'
                default_type_value = self.lineEdit_newAction_default_service_type.text()
                default_service = self.lineEdit_newAction_default_service.text()
                default_service_request = yaml.load(self.textEdit_newAction_default_service_request.toPlainText())
                default_service_response = yaml.load(self.textEdit_newAction_default_service_response.toPlainText())
                pddl_param_value = yaml.load('[' + self.lineEdit_newAction_service_pddlParam.text() + ']')
                fill = {'name': newAction_name_value,
                        'interface_type': interfaceType_value,
                        'default_service_type': default_type_value,
                        'default_service': default_service,
                        'default_service_request': default_service_request,
                        'default_service_response': default_service_response,
                        'pddl_parameters': pddl_param_value,
                        'parameter_values': self.popup_service.param_values_data
                        }
                # fill main data with new action entry
                self.internal_data.append(fill)
                self.popup_service.param_values_data = []  # parameter entry from popup service window

        elif origin == "add_state":
            # States in the fsms
            fill = {}
            addState_name_value = self.lineEdit_addState_name.text()
            if self.radioButton_addState_fsm.isChecked() == True:
                interfaceType_value = 'fsm'
                fill = {'name': addState_name_value,
                        'interface_type': interfaceType_value,
                        'states': []
                        }
                # Add the new internal data
                internal_data_entry_level = self.get_internal_data_entry_level()
                internal_data_entry_level['states'].append(fill)

            if self.radioButton_addState_actionlib.isChecked() == True:
                interfaceType_value = 'actionlib'
                default_msgType_value = self.lineEdit_addState_default_actionlib_msgType.text()
                default_topic_value = self.lineEdit_addState_default_actionlib_topic.text()
                default_goal_value = yaml.load(self.textEdit_addState_default_actionlib_goal.toPlainText())
                pddl_param_value = yaml.load('[' + self.lineEdit_addState_actionlib_pddlParam.text() + ']')
                succeeded_tostate = self.lineEdit_addState_actionlib_transition_suceeded_toState.text()
                succeeded_effects = self.lineEdit_addState_actionlib_transition_suceeded_effect.text()
                failed1_tostate = self.lineEdit_addState_actionlib_transition_failed1_toState.text()
                failed1_effects = self.lineEdit_addState_actionlib_transition_failed1_effect.text()
                failed2_tostate = self.lineEdit_addState_actionlib_transition_failed2_toState.text()
                failed2_effects = self.lineEdit_addState_actionlib_transition_failed2_effect.text()
                fill = {'name': addState_name_value,
                        'interface_type': interfaceType_value,
                        'default_actionlib_msg_type': default_msgType_value,
                        'default_actionlib_topic': default_topic_value,
                        'default_actionlib_goal': default_goal_value,
                        'pddl_parameters': pddl_param_value,
                        'parameter_values': self.popup_actionlib.param_values_data,
                        'transitions':
                            {'succeeded':
                                [
                                    {'to_state': succeeded_tostate,
                                     'effects': succeeded_effects}
                                ],
                                'failed':
                                    [
                                        {'to_state': failed1_tostate,
                                         'effects': failed1_effects},
                                        {'to_state': failed2_tostate,
                                         'effects': failed2_effects}
                                    ]
                            }
                        }
                self.popup_actionlib.param_values_data = []
                # Add the new internal data
                internal_data_entry_level = self.get_internal_data_entry_level()
                internal_data_entry_level['states'].append(fill)

            if self.radioButton_addState_service.isChecked() == True:
                interfaceType_value = 'service'
                default_type_value = self.lineEdit_addState_default_service_type.text()
                default_service = self.lineEdit_addState_default_service.text()
                default_service_request = yaml.load(self.textEdit_addState_default_service_request.toPlainText())
                default_service_response = yaml.load(self.textEdit_addState_default_service_response.toPlainText())
                pddl_param_value = yaml.load('[' + self.lineEdit_addState_service_pddlParam.text() + ']')
                succeeded_tostate = self.lineEdit_addState_service_transition_suceeded_toState.text()
                succeeded_effects = self.lineEdit_addState_service_transition_suceeded_effect.text()
                failed1_tostate = self.lineEdit_addState_service_transition_failed1_toState.text()
                failed1_effects = self.lineEdit_addState_service_transition_failed1_effect.text()
                failed2_tostate = self.lineEdit_addState_service_transition_failed2_toState.text()
                failed2_effects = self.lineEdit_addState_service_transition_failed2_effect.text()
                fill = {'name': addState_name_value,
                        'interface_type': interfaceType_value,
                        'default_service_type': default_type_value,
                        'default_service': default_service,
                        'default_service_request': default_service_request,
                        'default_service_response': default_service_response,
                        'pddl_parameters': pddl_param_value,
                        'parameter_values': self.popup_service.param_values_data,
                        'transitions':
                            {'succeeded':
                                [
                                    {'to_state': succeeded_tostate,
                                     'effects': succeeded_effects}
                                ],
                                'failed':
                                    [
                                        {'to_state': failed1_tostate,
                                         'effects': failed1_effects},
                                        {'to_state': failed2_tostate,
                                         'effects': failed2_effects}
                                    ]
                            }
                        }
                self.popup_service.param_values_data = []
                # Add the new internal data
                internal_data_entry_level = self.get_internal_data_entry_level()
                internal_data_entry_level['states'].append(fill)

    # Update internal data from GUI data
    def update_internal_data_from_gui_data(self, origin):
        internal_data_entry_level = self.get_internal_data_entry_level()

        if origin == "add_action":
            # get update from new action window
            self.stackedWidget_newAction_addState.setCurrentIndex(2)
            if internal_data_entry_level['interface_type'] == 'fsm':
                internal_data_entry_level['name'] = self.lineEdit_newAction_name.text()
                internal_data_entry_level['interface_type'] = 'fsm'
                self.stackedWidget_newAction_setting.setCurrentIndex(2)

            elif internal_data_entry_level['interface_type'] == 'actionlib':
                internal_data_entry_level['name'] = self.lineEdit_newAction_name.text()
                internal_data_entry_level['interface_type'] = 'actionlib'
                internal_data_entry_level['default_actionlib_msg_type'] = self.lineEdit_newAction_default_actionlib_msgType.text()
                internal_data_entry_level['default_actionlib_topic'] = self.lineEdit_newAction_default_actionlib_topic.text()
                internal_data_entry_level['default_actionlib_goal'] = yaml.load(self.textEdit_newAction_default_actionlib_goal.toPlainText())
                internal_data_entry_level['pddl_parameters'] = yaml.load('[' + self.lineEdit_newAction_actionlib_pddlParam.text() + ']')
                internal_data_entry_level['parameter_values'] = self.popup_actionlib.param_values_data
                self.stackedWidget_newAction_setting.setCurrentIndex(0)

            elif internal_data_entry_level['interface_type'] == 'service':
                internal_data_entry_level['name'] = self.lineEdit_newAction_name.text()
                internal_data_entry_level['interface_type'] = 'service'
                internal_data_entry_level['default_service_type'] = self.lineEdit_newAction_default_service_type.text()
                internal_data_entry_level['default_service'] = self.lineEdit_newAction_default_service.text()
                internal_data_entry_level['default_service_request'] = yaml.load(self.textEdit_newAction_default_service_request.toPlainText())
                internal_data_entry_level['default_service_response'] = yaml.load(self.textEdit_newAction_default_service_response.toPlainText())
                internal_data_entry_level['pddl_parameters'] = yaml.load('[' + self.lineEdit_newAction_service_pddlParam.text() + ']')
                internal_data_entry_level['parameter_values'] = self.popup_service.param_values_data
                self.stackedWidget_newAction_setting.setCurrentIndex(1)

        elif origin == "add_state":
            # get update from add state window
            self.stackedWidget_newAction_addState.setCurrentIndex(0)
            if internal_data_entry_level['interface_type'] == 'fsm':
                internal_data_entry_level['name'] = self.lineEdit_addState_name.text()
                internal_data_entry_level['interface_type'] = 'fsm'
                self.stackedWidget_addState_setting.setCurrentIndex(2)

            elif internal_data_entry_level['interface_type'] == 'actionlib':
                internal_data_entry_level['name'] = self.lineEdit_addState_name.text()
                internal_data_entry_level['interface_type'] = 'actionlib'
                internal_data_entry_level['default_actionlib_msg_type'] = self.lineEdit_addState_default_actionlib_msgType.text()
                internal_data_entry_level['default_actionlib_topic'] = self.lineEdit_addState_default_actionlib_topic.text()
                internal_data_entry_level['default_actionlib_goal'] = yaml.load(self.textEdit_addState_default_actionlib_goal.toPlainText())
                internal_data_entry_level['pddl_parameters'] = yaml.load('[' + self.lineEdit_addState_actionlib_pddlParam.text() + ']')
                internal_data_entry_level['parameter_values'] = self.popup_actionlib.param_values_data
                success1 = {'to_state': self.lineEdit_addState_actionlib_transition_suceeded_toState.text(),
                            'effects': self.lineEdit_addState_actionlib_transition_suceeded_effect.text()}
                internal_data_entry_level['transitions']['succeeded'] = []
                internal_data_entry_level['transitions']['succeeded'].append(success1)
                failed1 = {'to_state': self.lineEdit_addState_actionlib_transition_failed1_toState.text(),
                            'effects': self.lineEdit_addState_actionlib_transition_failed1_effect.text()}
                failed2 = {'to_state': self.lineEdit_addState_actionlib_transition_failed2_toState.text(),
                           'effects': self.lineEdit_addState_actionlib_transition_failed2_effect.text()}
                internal_data_entry_level['transitions']['failed'] = []
                internal_data_entry_level['transitions']['failed'].append(failed1)
                internal_data_entry_level['transitions']['failed'].append(failed2)
                self.stackedWidget_addState_setting.setCurrentIndex(0)

            elif internal_data_entry_level['interface_type'] == 'service':
                internal_data_entry_level['name'] = self.lineEdit_addState_name.text()
                internal_data_entry_level['interface_type'] = 'service'
                internal_data_entry_level['default_service_type'] = self.lineEdit_addState_default_service_type.text()
                internal_data_entry_level['default_service'] = self.lineEdit_addState_default_service.text()
                internal_data_entry_level['default_service_request'] = yaml.load(
                    self.textEdit_addState_default_service_request.toPlainText())
                internal_data_entry_level['default_service_response'] = yaml.load(
                    self.textEdit_addState_default_service_response.toPlainText())
                internal_data_entry_level['pddl_parameters'] = yaml.load('[' + self.lineEdit_addState_service_pddlParam.text() + ']')
                internal_data_entry_level['parameter_values'] = self.popup_service.param_values_data
                success1 = {'to_state': self.lineEdit_addState_service_transition_suceeded_toState.text(),
                            'effects': self.lineEdit_addState_service_transition_suceeded_effect.text()}
                internal_data_entry_level['transitions']['succeeded'] = []
                internal_data_entry_level['transitions']['succeeded'].append(success1)
                failed1 = {'to_state': self.lineEdit_addState_service_transition_failed1_toState.text(),
                           'effects': self.lineEdit_addState_service_transition_failed1_effect.text()}
                failed2 = {'to_state': self.lineEdit_addState_service_transition_failed2_toState.text(),
                           'effects': self.lineEdit_addState_service_transition_failed2_effect.text()}
                internal_data_entry_level['transitions']['failed'] = []
                internal_data_entry_level['transitions']['failed'].append(failed1)
                internal_data_entry_level['transitions']['failed'].append(failed2)

                self.stackedWidget_addState_setting.setCurrentIndex(1)

    # Set GUI data from internal data
    def set_gui_data_from_internal_data(self, origin):
        internal_data_entry_level = self.get_internal_data_entry_level()

        if origin == "add_action":
            # if selected action is no state machine(fsm) then refill new action window
            self.stackedWidget_newAction_addState.setCurrentIndex(2)
            if internal_data_entry_level['interface_type'] == 'fsm':
                self.lineEdit_newAction_name.setText(internal_data_entry_level['name'])
                self.radioButton_newAction_fsm.setChecked(True)
                self.radioButton_newAction_actionlib.setChecked(False)
                self.radioButton_newAction_service.setChecked(False)
                self.stackedWidget_newAction_setting.setCurrentIndex(2)

            elif internal_data_entry_level['interface_type'] == 'actionlib':
                self.lineEdit_newAction_name.setText(internal_data_entry_level['name'])
                self.radioButton_newAction_fsm.setChecked(False)
                self.radioButton_newAction_actionlib.setChecked(True)
                self.radioButton_newAction_service.setChecked(False)
                self.lineEdit_newAction_default_actionlib_msgType.setText(internal_data_entry_level.get('default_actionlib_msg_type', ''))
                self.lineEdit_newAction_default_actionlib_topic.setText(internal_data_entry_level.get('default_actionlib_topic', ''))
                default_actionlib_goal = internal_data_entry_level.get('default_actionlib_goal', None)
                if default_actionlib_goal is None:
                    self.textEdit_newAction_default_actionlib_goal.setText("")
                else:
                    self.textEdit_newAction_default_actionlib_goal.setText(yaml.dump(default_actionlib_goal,
                                                                                     default_flow_style=False))
                pddl_parameters = internal_data_entry_level.get('pddl_parameters', None)
                if pddl_parameters is None:
                    self.lineEdit_newAction_actionlib_pddlParam.setText("")
                else:
                    self.lineEdit_newAction_actionlib_pddlParam.setText(yaml.dump(pddl_parameters)[1:-2])
                param_values = internal_data_entry_level['parameter_values']
                self.tableView = self.tableView_newAction_actionlib_paramValues
                self.tableView.selectionModel().reset()
                self.tableView_model_newAction_actionlib.removeRows(0, self.tableView_model_newAction_actionlib.rowCount())
                self.view_model = self.tableView_model_newAction_actionlib
                self.fill_params_tableView(param_values)
                self.popup_actionlib.param_values_data = param_values
                self.stackedWidget_newAction_setting.setCurrentIndex(0)

            elif internal_data_entry_level['interface_type'] == 'service':
                self.lineEdit_newAction_name.setText(internal_data_entry_level['name'])
                self.radioButton_newAction_fsm.setChecked(False)
                self.radioButton_newAction_actionlib.setChecked(False)
                self.radioButton_newAction_service.setChecked(True)
                self.lineEdit_newAction_default_service_type.setText(internal_data_entry_level.get('default_service_type', ''))
                self.lineEdit_newAction_default_service.setText(internal_data_entry_level.get('default_service', ''))
                default_service_request = internal_data_entry_level.get('default_service_request', None)
                default_service_response = internal_data_entry_level.get('default_service_response', None)
                if default_service_request is None:
                    self.textEdit_newAction_default_service_request.setText("")
                else:
                    self.textEdit_newAction_default_service_request.setText(yaml.dump(default_service_request,
                                                                                   default_flow_style=False))
                if default_service_response is None:
                    self.textEdit_newAction_default_service_response.setText("")
                else:
                    self.textEdit_newAction_default_service_response.setText(yaml.dump(default_service_response,
                                                                                       default_flow_style=False))
                pddl_parameters = internal_data_entry_level.get('pddl_parameters', None)
                if pddl_parameters is None:
                    self.lineEdit_newAction_service_pddlParam.setText("")
                else:
                    self.lineEdit_newAction_service_pddlParam.setText(yaml.dump(pddl_parameters)[1:-2])
                param_values = internal_data_entry_level['parameter_values']
                self.tableView = self.tableView_newAction_service_paramValues
                self.tableView.selectionModel().reset()
                self.tableView_model_newAction_service.removeRows(0, self.tableView_model_newAction_service.rowCount())
                self.view_model = self.tableView_model_newAction_service
                self.fill_params_tableView(param_values)
                self.popup_service.param_values_data = param_values
                self.stackedWidget_newAction_setting.setCurrentIndex(1)

        elif origin == "add_state":
            # else fill add state window
            self.stackedWidget_newAction_addState.setCurrentIndex(0)
            if internal_data_entry_level['interface_type'] == 'fsm':
                self.radioButton_addState_actionlib.setChecked(False)
                self.radioButton_addState_service.setChecked(False)
                self.radioButton_addState_fsm.setChecked(True)
                self.lineEdit_addState_name.setText(internal_data_entry_level['name'])
                self.stackedWidget_addState_setting.setCurrentIndex(2)

            elif internal_data_entry_level['interface_type'] == 'actionlib':
                self.lineEdit_addState_name.setText(internal_data_entry_level['name'])
                self.radioButton_addState_actionlib.setChecked(True)
                self.radioButton_addState_service.setChecked(False)
                self.radioButton_addState_fsm.setChecked(False)
                self.lineEdit_addState_default_actionlib_msgType.setText(internal_data_entry_level.get('default_actionlib_msg_type', ''))
                self.lineEdit_addState_default_actionlib_topic.setText(internal_data_entry_level.get('default_actionlib_topic', ''))
                default_actionlib_goal = internal_data_entry_level.get('default_actionlib_goal', None)
                if default_actionlib_goal is None:
                    self.textEdit_addState_default_actionlib_goal.setText("")
                else:
                    self.textEdit_addState_default_actionlib_goal.setText(yaml.dump(default_actionlib_goal,
                                                                                    default_flow_style=False))
                pddl_parameters = internal_data_entry_level.get('pddl_parameters', None)
                if pddl_parameters is None:
                    self.lineEdit_addState_actionlib_pddlParam.setText("")
                else:
                    self.lineEdit_addState_actionlib_pddlParam.setText(yaml.dump(pddl_parameters)[1:-2])
                param_values = internal_data_entry_level.get('parameter_values', [])
                self.tableView = self.tableView_addState_actionlib_paramValues
                self.tableView_model_addState_actionlib.removeRows(0, self.tableView_model_addState_actionlib.rowCount())
                self.view_model = self.tableView_model_addState_actionlib
                self.fill_params_tableView(param_values)
                self.popup_actionlib.param_values_data = param_values

                self.lineEdit_addState_actionlib_transition_suceeded_toState.setText('')
                self.lineEdit_addState_actionlib_transition_suceeded_effect.setText('')
                self.lineEdit_addState_actionlib_transition_failed1_toState.setText('')
                self.lineEdit_addState_actionlib_transition_failed1_effect.setText('')
                self.lineEdit_addState_actionlib_transition_failed2_toState.setText('')
                self.lineEdit_addState_actionlib_transition_failed2_effect.setText('')

                if internal_data_entry_level.has_key('transitions'):
                    if len(internal_data_entry_level['transitions']['succeeded']) == 1:
                        self.lineEdit_addState_actionlib_transition_suceeded_toState.setText(
                            internal_data_entry_level['transitions']['succeeded'][0]['to_state'])
                        self.lineEdit_addState_actionlib_transition_suceeded_effect.setText(
                            internal_data_entry_level['transitions']['succeeded'][0]['effects'])
                    if len(internal_data_entry_level['transitions']['failed']) >= 1:
                        self.lineEdit_addState_actionlib_transition_failed1_toState.setText(
                            internal_data_entry_level['transitions']['failed'][0]['to_state'])
                        self.lineEdit_addState_actionlib_transition_failed1_effect.setText(
                            internal_data_entry_level['transitions']['failed'][0]['effects'])
                    if len(internal_data_entry_level['transitions']['failed']) == 2:
                        self.lineEdit_addState_actionlib_transition_failed2_toState.setText(
                            internal_data_entry_level['transitions']['failed'][1]['to_state'])
                        self.lineEdit_addState_actionlib_transition_failed2_effect.setText(
                            internal_data_entry_level['transitions']['failed'][1]['effects'])

                self.stackedWidget_addState_setting.setCurrentIndex(0)

            elif internal_data_entry_level['interface_type'] == 'service':
                self.lineEdit_addState_name.setText(internal_data_entry_level['name'])
                self.radioButton_addState_actionlib.setChecked(False)
                self.radioButton_addState_service.setChecked(True)
                self.radioButton_addState_fsm.setChecked(False)
                self.lineEdit_addState_default_service_type.setText(internal_data_entry_level.get('default_service_type', ''))
                self.lineEdit_addState_default_service.setText(internal_data_entry_level.get('default_service', ''))
                default_service_request = internal_data_entry_level.get('default_service_request', None)
                default_service_response = internal_data_entry_level.get('default_service_response', None)
                if default_service_request is None:
                    self.textEdit_addState_default_service_request.setText("")
                else:
                    self.textEdit_addState_default_service_request.setText(yaml.dump(default_service_request,
                                                                                  default_flow_style=False))
                if default_service_response is None:
                    self.textEdit_addState_default_service_response.setText("")
                else:
                    self.textEdit_addState_default_service_response.setText(yaml.dump(default_service_response,
                                                                                      default_flow_style=False))
                pddl_parameters = internal_data_entry_level.get('pddl_parameters', None)
                if pddl_parameters is None:
                    self.lineEdit_addState_service_pddlParam.setText("")
                else:
                    self.lineEdit_addState_service_pddlParam.setText(yaml.dump(pddl_parameters)[1:-2])
                param_values = internal_data_entry_level.get('parameter_values', [])
                self.tableView = self.tableView_addState_service_paramValues
                self.tableView_model_addState_service.removeRows(0, self.tableView_model_addState_service.rowCount())
                self.view_model = self.tableView_model_addState_service
                self.fill_params_tableView(param_values)
                self.popup_service.param_values_data = param_values

                self.lineEdit_addState_service_transition_suceeded_toState.setText('')
                self.lineEdit_addState_service_transition_suceeded_effect.setText('')
                self.lineEdit_addState_service_transition_failed1_toState.setText('')
                self.lineEdit_addState_service_transition_failed1_effect.setText('')
                self.lineEdit_addState_service_transition_failed2_toState.setText('')
                self.lineEdit_addState_service_transition_failed2_effect.setText('')

                if internal_data_entry_level.has_key('transitions'):
                    if len(internal_data_entry_level['transitions']['succeeded']) == 1:
                        self.lineEdit_addState_service_transition_suceeded_toState.setText(
                            internal_data_entry_level['transitions']['succeeded'][0]['to_state'])
                        self.lineEdit_addState_service_transition_suceeded_effect.setText(
                            internal_data_entry_level['transitions']['succeeded'][0]['effects'])
                    if len(internal_data_entry_level['transitions']['failed']) >= 1:
                        self.lineEdit_addState_service_transition_failed1_toState.setText(
                            internal_data_entry_level['transitions']['failed'][0]['to_state'])
                        self.lineEdit_addState_service_transition_failed1_effect.setText(
                            internal_data_entry_level['transitions']['failed'][0]['effects'])
                    if len(internal_data_entry_level['transitions']['failed']) == 2:
                        self.lineEdit_addState_service_transition_failed2_toState.setText(
                            internal_data_entry_level['transitions']['failed'][1]['to_state'])
                        self.lineEdit_addState_service_transition_failed2_effect.setText(
                            internal_data_entry_level['transitions']['failed'][1]['effects'])
                self.stackedWidget_addState_setting.setCurrentIndex(1)

    # -----------------------------------------------------------------------------------
    # -------------------- Windows resets and clears ------------------------------------
    # -----------------------------------------------------------------------------------

    def reset_treeView_overview(self):
        self.treeView_overview.blockSignals(True)
        self.treeView_overview.selectionModel().reset()
        self.model_overview.removeRows(0, self.model_overview.rowCount())
        self.treeView_overview.blockSignals(False)
        self.fill_model_overview(self.model_overview.invisibleRootItem(), self.internal_data, 0)

    # Fill overview recursively with self.internal_data
    def fill_model_overview(self, parent, entry, counter):
        if isinstance(entry, list):
            for i in range(len(entry)):
                if not entry[i]: # check if dictionary is empty
                    pass
                else:
                    name = entry[i]['name']
                    type = entry[i]['interface_type']
                    it1 = QStandardItem(str(counter))
                    it2 = QStandardItem(str(name))
                    if type == "fsm":
                        it3 = QStandardItem("fsm")
                    elif type == "actionlib":
                        it3 = QStandardItem("act")
                    if type == "service":
                        it3 = QStandardItem("srv")

                    parent.appendRow([it1, it2, it3])
                    if entry[i].has_key('states'):
                        child_count = 0
                        self.fill_model_overview(it1,entry[i]['states'], child_count)

                    counter += 1

    def clear_new_action_input(self):
        self.lineEdit_newAction_name.clear()
        # fsm
        self.radioButton_newAction_fsm.setChecked(False)

        # actionlib
        self.radioButton_newAction_actionlib.setChecked(False)
        self.lineEdit_newAction_default_actionlib_msgType.clear()
        self.lineEdit_newAction_default_actionlib_topic.clear()
        self.textEdit_newAction_default_actionlib_goal.clear()
        self.lineEdit_newAction_actionlib_pddlParam.clear()
        self.tableView_newAction_actionlib_paramValues.selectionModel().reset()
        self.tableView_model_newAction_actionlib.removeRows(0, self.tableView_model_newAction_actionlib.rowCount())

        # service
        self.radioButton_newAction_service.setChecked(False)
        self.lineEdit_newAction_default_service_type.clear()
        self.lineEdit_newAction_default_service.clear()
        self.textEdit_newAction_default_service_request.clear()
        self.textEdit_newAction_default_service_response.clear()
        self.lineEdit_newAction_service_pddlParam.clear()
        self.tableView_newAction_service_paramValues.selectionModel().reset()
        self.tableView_model_newAction_service.removeRows(0, self.tableView_model_newAction_service.rowCount())

    def clear_add_state_input(self):
        self.lineEdit_addState_name.clear()

        # fsm
        self.radioButton_addState_fsm.setChecked(False)

        # actionlib
        self.radioButton_addState_actionlib.setChecked(False)
        self.lineEdit_addState_default_actionlib_msgType.clear()
        self.lineEdit_addState_default_actionlib_topic.clear()
        self.textEdit_addState_default_actionlib_goal.clear()
        self.lineEdit_addState_actionlib_pddlParam.clear()
        self.lineEdit_addState_actionlib_transition_suceeded_toState.clear()
        self.lineEdit_addState_actionlib_transition_suceeded_effect.clear()
        self.lineEdit_addState_actionlib_transition_failed1_toState.clear()
        self.lineEdit_addState_actionlib_transition_failed1_effect.clear()
        self.lineEdit_addState_actionlib_transition_failed2_toState.clear()
        self.lineEdit_addState_actionlib_transition_failed2_effect.clear()
        self.tableView_addState_actionlib_paramValues.selectionModel().reset()
        self.tableView_model_addState_actionlib.removeRows(0, self.tableView_model_addState_actionlib.rowCount())

        # service
        self.radioButton_addState_service.setChecked(False)
        self.lineEdit_addState_default_service_type.clear()
        self.lineEdit_addState_default_service.clear()
        self.textEdit_addState_default_service_request.clear()
        self.textEdit_addState_default_service_response.clear()
        self.lineEdit_addState_service_pddlParam.clear()
        self.lineEdit_addState_service_transition_suceeded_toState.clear()
        self.lineEdit_addState_service_transition_suceeded_effect.clear()
        self.lineEdit_addState_service_transition_failed1_toState.clear()
        self.lineEdit_addState_service_transition_failed1_effect.clear()
        self.lineEdit_addState_service_transition_failed2_toState.clear()
        self.lineEdit_addState_service_transition_failed2_effect.clear()
        self.tableView_addState_service_paramValues.selectionModel().reset()
        self.tableView_model_addState_service.removeRows(0, self.tableView_model_addState_service.rowCount())

        # Popups
        self.popup_actionlib.param_values_data = []
        self.popup_service.param_values_data = []

    def fill_params_tableView(self, param_values):
        for param_value in param_values:  # go through the umber of parameter sets
            text = ""
            if param_value['values'] is not None:
                text += yaml.dump(param_value['values'])[1:-2]
            it = QStandardItem(text)
            self.view_model.appendRow(it)  # append text with corresponding number i for each seat on tableview

    # -----------------------------------------------------------------------------------
    # -------------------- Further Utilities --------------------------------------------
    # -----------------------------------------------------------------------------------

    #remove all empty dictionary and empty entries
    def strip_empties_from_list(self, data):
        new_data = []
        for v in data: #go through every index in data
            if isinstance(v, dict):
                v = self.strip_empties_from_dict(v) # remove empty dictionaries
            elif isinstance(v, list):
                v = self.strip_empties_from_list(v) # remove any empty list
            if v not in (None, str(), list(), dict(),): # append only data with entries
                new_data.append(v)
        self.new_data = new_data
        return self.new_data

    #remove all empty list and empty entries
    def strip_empties_from_dict(self, data):
        new_data = {}
        for k, v in data.items(): # go theough every key and value in dictionary
            if isinstance(v, dict):
                v = self.strip_empties_from_dict(v)
            elif isinstance(v, list):
                v = self.strip_empties_from_list(v)
            if v not in (None, str(), list(), dict(),):
                new_data[k] = v
        return new_data

    def check_state_name_present_in_fsm_states(self, state_name):
        current_index = self.get_current_index()

        # Check if the state name is already used in the selected fsm
        i_child = 0
        state_name_found = False
        while current_index.child(i_child, 1).isValid():
            if current_index.child(i_child, 1) == state_name:
                state_name_found = True
                break
            i_child += 1
        return state_name_found
