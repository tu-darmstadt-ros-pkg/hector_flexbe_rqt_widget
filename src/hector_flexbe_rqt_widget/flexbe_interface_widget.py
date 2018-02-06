#!/usr/bin/env python

import os

import rospy
import rospkg
import actionlib

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, pyqtSignal
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QPushButton, QTableWidgetItem, QHeaderView, QHBoxLayout

from std_msgs.msg import Empty, Bool, UInt8, String
from flexbe_core import BehaviorLibrary
from flexbe_msgs.msg import BEStatus, BehaviorExecutionAction, BehaviorExecutionGoal


class FlexBeInterfacePlugin(Plugin):

    def __init__(self, context):
        super(FlexBeInterfacePlugin, self).__init__(context)
        self_context = context

        self._widget = FlexBeWidget(self, context)

        if self_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % self_context.serial_number()))

        self_context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()


class FlexBeWidget(QWidget):

    updateCurrentBehavior = pyqtSignal(str)
    updateCurrentState = pyqtSignal(str)
    updateStateLog = pyqtSignal(str)
    clearStateLog = pyqtSignal()
    setPauseButtonText = pyqtSignal(str)

    def __init__(self, parent, context):
        super(FlexBeWidget, self).__init__()

        self._attached = False
        self._paused = False

        # load from ui
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('hector_flexbe_rqt_widget'), 'resource', 'flexbe_interface.ui')
        loadUi(ui_file, self, {'QWidget': QWidget})
        self.setObjectName('FlexBeInterfaceUi')

        # connect to signals
        self.executePushButton.clicked[bool].connect(self._execute_button_pressed)
        self.pausePushButton.clicked[bool].connect(self._pause_button_pressed)
        self.abortPushButton.clicked[bool].connect(self._abort_button_pressed)
        self.attachPushButton.clicked[bool].connect(self._attach_button_pressed)
        self.autonomyComboBox.currentIndexChanged[int].connect(lambda index: self._autonomy_pub.publish(index))
        self.argumentTableWidget.itemChanged[QTableWidgetItem].connect(lambda item: self._table_widget_enter_pressed(self.argumentTableWidget, item))
        self.userdataTableWidget.itemChanged[QTableWidgetItem].connect(lambda item: self._table_widget_enter_pressed(self.userdataTableWidget, item))
        self.clearArgumentPushButton.clicked[bool].connect(lambda: self._clear_table_widget(self.argumentTableWidget))
        self.clearUserdataPushButton.clicked[bool].connect(lambda: self._clear_table_widget(self.userdataTableWidget))
        self.clearLogPushButton.clicked[bool].connect(self.logTextEdit.clear)
        self.saveLogPushButton.clicked[bool].connect(self._save_log)

        # Qt signals
        self.updateCurrentBehavior.connect(self.behaviorLineEdit.setText)
        self.updateCurrentState.connect(self.currentStateLineEdit.setText)
        self.setPauseButtonText.connect(self.pausePushButton.setText)
        self.updateStateLog.connect(self.logTextEdit.append)
        self.clearStateLog.connect(self.logTextEdit.clear)

        # init table widgets
        self._init_table_widget(self.argumentTableWidget)
        self._init_table_widget(self.userdataTableWidget)

        # init behavior list
        self._lib = BehaviorLibrary()
        self._retrieve_behaviors()

        # init subscribers
        self._status_sub = rospy.Subscriber('flexbe/status', BEStatus, self._status_cb)
        self._state_update_sub = rospy.Subscriber('flexbe/debug/current_state', String, self._state_update_cb)
        self._behavior_update_sub = rospy.Subscriber('flexbe/behavior_update', String, self._behavior_update_cb)

        # init publisher
        self._attach_pub = rospy.Publisher('flexbe/command/attach', UInt8, queue_size=1)
        self._autonomy_pub = rospy.Publisher('flexbe/command/autonomy', UInt8, queue_size=1)
        self._command_pause_pub = rospy.Publisher('flexbe/command/pause', Bool, queue_size=1)
        self._command_preempt_pub = rospy.Publisher('flexbe/command/preempt', Empty, queue_size=1)

        # init action client
        self._execute_behavior_client = actionlib.SimpleActionClient('flexbe/execute_behavior', BehaviorExecutionAction)

    def shutdown_plugin(self):
        print('Shutting down ...')
        self._status_sub.unregister()
        self._state_update_sub.unregister()
        self._behavior_update_sub.unregister()
        print('Done!')

    def _status_cb(self, status):
        if status.code in [BEStatus.FINISHED, BEStatus.FAILED, BEStatus.READY, BEStatus.ERROR]:
            self.executePushButton.setEnabled(True)
            self.setPauseButtonText.emit('Pause')
            self.abortPushButton.setEnabled(False)
            self.attachPushButton.setEnabled(False)
            self._detach()
            self._paused = False
        else:
            self._retrieve_behavior_name(status.behavior_id)
            self.executePushButton.setEnabled(False)
            self.abortPushButton.setEnabled(True)
            self.attachPushButton.setEnabled(True)

        if status.code == BEStatus.STARTED:
            self.updateCurrentState.emit('Started')
        if status.code == BEStatus.FINISHED:
            self.updateCurrentState.emit('Finished')
        if status.code == BEStatus.FAILED:
            self.updateCurrentState.emit('Failed')
        if status.code == BEStatus.LOCKED:
            self.updateCurrentState.emit('Locked')
        if status.code == BEStatus.WAITING:
            self.updateCurrentState.emit('Waiting')
        if status.code == BEStatus.SWITCHING:
            self.updateCurrentState.emit('Switching')
        if status.code == BEStatus.WARNING:
            self.updateCurrentState.emit('Warning')
        if status.code == BEStatus.ERROR:
            self.updateCurrentState.emit('Error')
        if status.code == BEStatus.READY:
            self.updateCurrentState.emit('Ready')

    def _state_update_cb(self, msg):
        if self._attached:
            self.updateStateLog.emit(msg.data)

    def _behavior_update_cb(self, msg):
        if self._attached:
            self.updateCurrentState.emit(msg.data)

    def _execute_behavior_active_cb(self):
        self.clearStateLog.emit()

    def _retrieve_behaviors(self):
        names = []
        for item in self._lib._behavior_lib.values():
            names.append(item['name'])
        self._update_behaviors(names)

    def _retrieve_behavior_name(self, id):
        #behavior = self._lib.get_behavior(id)  # TODO: How to get name?
        self.updateCurrentBehavior.emit('N/A')

    def _execute_button_pressed(self):
        if not self.behaviorsListWidget.currentItem():
            return

        if not self._execute_behavior_client.wait_for_server(rospy.Duration(1.0)):
            rospy.logerr('FlexBE action server not available!')
            self.updateCurrentState.emit('FlexBE action server not available!')
            return

        goal = BehaviorExecutionGoal()
        goal.behavior_name = self.behaviorsListWidget.currentItem().text()

        # add behavior arguments
        for row in range(0, self.argumentTableWidget.rowCount()):
            key = self.argumentTableWidget.item(row, 0).text()
            value = self.argumentTableWidget.item(row, 1).text()
            if key and value:
                goal.arg_keys.append(key)
                goal.arg_values.append(value)

        # add initial userdata
        for row in range(0, self.userdataTableWidget.rowCount()):
            key = self.userdataTableWidget.item(row, 0).text()
            value = self.userdataTableWidget.item(row, 1).text()
            if key and value:
                goal.input_keys.append(key)
                goal.input_values.append(value)

        self._execute_behavior_client.send_goal(goal, active_cb=self._execute_behavior_active_cb)

    def _pause_button_pressed(self):
        self._paused = not self._paused
        self._command_pause_pub.publish(self._paused)

        if self._paused:
            self.updateCurrentState.emit('Paused')
            self.setPauseButtonText.emit('Resume')
        else:
            self.updateCurrentState.emit('N/A')
            self.setPauseButtonText.emit('Pause')

    def _abort_button_pressed(self):
        self._execute_behavior_client.cancel_goal()
        self._command_preempt_pub.publish()
        self.updateCurrentState.emit('Canceled')

    def _attach_button_pressed(self):
        if self._attached:
            self._detach()
        else:
            self._attach()

    def _attach(self):
        self._attached = True
        self.pausePushButton.setEnabled(True)
        self.attachPushButton.setText('Detach')
        self.autonomyLabel.setEnabled(True)
        self.autonomyComboBox.setEnabled(True)
        self._attach_pub.publish(1)  # attach to state machine
        self._autonomy_pub.publish(self.autonomyComboBox.currentIndex())

    def _detach(self):
        self._attached = False
        self.pausePushButton.setEnabled(False)
        self.attachPushButton.setText('Attach')
        self.autonomyLabel.setEnabled(False)
        self.autonomyComboBox.setEnabled(False)
        self.updateCurrentBehavior.emit('N/A')
        self.updateCurrentState.emit('N/A')
        #self._attach_pub.publish(0)  # detach from state machine; TODO: Seems not to work

    def _update_behaviors(self, names):
        enabled = self.executePushButton.isEnabled()
        self.executePushButton.setEnabled(False)
        self.behaviorsListWidget.clear()
        self.behaviorsListWidget.addItems(names)
        self.behaviorsListWidget.sortItems()
        self.executePushButton.setEnabled(enabled and self.behaviorsListWidget.count() > 0)

    def _create_cell_push_button(self, text, clicked_cb, icon=None):
        widget = QWidget()

        button = QPushButton()
        button.setText(text)
        if icon:
            button.setIcon(icon)
        button.clicked[bool].connect(clicked_cb)

        hlayout = QHBoxLayout(widget)
        hlayout.addWidget(button)
        hlayout.setAlignment(Qt.AlignCenter)
        hlayout.setContentsMargins(0, 0, 0, 0)

        widget.setLayout(hlayout)
        return widget

    def _init_table_widget(self, tableWidget):
        tableWidget.blockSignals(True)

        tableWidget.setRowCount(1)
        tableWidget.setColumnCount(3)

        tableWidget.verticalHeader().setVisible(False)
        tableWidget.setHorizontalHeaderLabels(['Key', 'Value', ''])
        tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tableWidget.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)

        tableWidget.setItem(0, 0, QTableWidgetItem())
        tableWidget.setItem(0, 1, QTableWidgetItem())
        tableWidget.setItem(0, 2, QTableWidgetItem())
        tableWidget.item(0, 2).setFlags(Qt.NoItemFlags)
        tableWidget.setCellWidget(0, 2, self._create_cell_push_button('', lambda: self._add_entry(tableWidget), QIcon.fromTheme('list-add')))

        tableWidget.blockSignals(False)

    def _clear_table_widget(self, tableWidget):
        tableWidget.setRowCount(1)
        self._init_table_widget(tableWidget)

    def _add_entry(self, tableWidget):
        tableWidget.blockSignals(True)

        # insert new row
        tableWidget.insertRow(tableWidget.rowCount()-1)

        # move key and value items up
        tableWidget.setItem(tableWidget.rowCount()-2, 0, tableWidget.takeItem(tableWidget.rowCount()-1, 0))
        tableWidget.setItem(tableWidget.rowCount()-2, 1, tableWidget.takeItem(tableWidget.rowCount()-1, 1))
        tableWidget.setItem(tableWidget.rowCount()-1, 0, QTableWidgetItem())
        tableWidget.setItem(tableWidget.rowCount()-1, 1, QTableWidgetItem())

        # add 'remove' button
        item = QTableWidgetItem()
        item.setFlags(Qt.NoItemFlags)
        tableWidget.setItem(tableWidget.rowCount()-2, 2, item)
        tableWidget.setCellWidget(tableWidget.rowCount()-2, 2,
                                  self._create_cell_push_button('', lambda: tableWidget.removeRow(item.row()), QIcon.fromTheme('list-remove')))

        tableWidget.blockSignals(False)

    def _table_widget_enter_pressed(self, tableWidget, item):
        if item.column() == 0:
            index = tableWidget.indexFromItem(tableWidget.item(item.row(), item.column()+1))
        elif item.row() < tableWidget.rowCount()-1:
            index = tableWidget.indexFromItem(tableWidget.item(item.row()+1, item.column()-1))
        else:
            index = tableWidget.indexFromItem(tableWidget.item(0, 0))

        tableWidget.setCurrentIndex(index)
        tableWidget.edit(index)

    def _save_log(self):
        dialog = QFileDialog()
        dialog.setAcceptMode(QFileDialog.AcceptSave)
        dialog.setNameFilter('Text file (*.txt)')
        dialog.setDefaultSuffix('.txt')
        dialog.setDirectory(os.getenv('HOME'))

        if dialog.exec_():
            file = open(dialog.selectedFiles()[0], 'w')
            file.write(self.logTextEdit.toPlainText())
            file.close()
