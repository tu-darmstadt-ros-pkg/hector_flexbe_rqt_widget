#!/usr/bin/env python

from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QWidget, QPushButton, QTableWidgetItem, QHeaderView, QHBoxLayout


class TableWidget():

    def __init__(self, table_widget):
        self._table_widget = table_widget

        # connect to signals
        self._table_widget.itemChanged[QTableWidgetItem].connect(self._enter_pressed)

        # init table widgets
        self._init_table_widget()

    def clear(self):
        self._table_widget.setRowCount(1)
        self._init_table_widget()

    def get_entries(self):
        params = {}
        for row in range(0, self._table_widget.rowCount()):
            key = self._table_widget.item(row, 0).text()
            value = self._table_widget.item(row, 1).text()
            if key and value:
                params[key] = value
        return params

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

    def _init_table_widget(self):
        self._table_widget.blockSignals(True)

        self._table_widget.setRowCount(1)
        self._table_widget.setColumnCount(3)

        self._table_widget.verticalHeader().setVisible(False)
        self._table_widget.setHorizontalHeaderLabels(['Key', 'Value', ''])
        self._table_widget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._table_widget.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)

        self._table_widget.setItem(0, 0, QTableWidgetItem())
        self._table_widget.setItem(0, 1, QTableWidgetItem())
        self._table_widget.setItem(0, 2, QTableWidgetItem())
        self._table_widget.item(0, 2).setFlags(Qt.NoItemFlags)
        self._table_widget.setCellWidget(0, 2, self._create_cell_push_button('', lambda: self._add_entry(), QIcon.fromTheme('list-add')))

        self._table_widget.blockSignals(False)

    def _add_entry(self):
        self._table_widget.blockSignals(True)

        # insert new row
        self._table_widget.insertRow(self._table_widget.rowCount()-1)

        # move key and value items up
        self._table_widget.setItem(self._table_widget.rowCount()-2, 0, self._table_widget.takeItem(self._table_widget.rowCount()-1, 0))
        self._table_widget.setItem(self._table_widget.rowCount()-2, 1, self._table_widget.takeItem(self._table_widget.rowCount()-1, 1))
        self._table_widget.setItem(self._table_widget.rowCount()-1, 0, QTableWidgetItem())
        self._table_widget.setItem(self._table_widget.rowCount()-1, 1, QTableWidgetItem())

        # add 'remove' button
        item = QTableWidgetItem()
        item.setFlags(Qt.NoItemFlags)
        self._table_widget.setItem(self._table_widget.rowCount()-2, 2, item)
        self._table_widget.setCellWidget(self._table_widget.rowCount()-2, 2,
                                         self._create_cell_push_button('', lambda: self._table_widget.removeRow(item.row()), QIcon.fromTheme('list-remove')))

        self._table_widget.blockSignals(False)

    def _enter_pressed(self, item):
        if item.column() == 0:
            index = self._table_widget.indexFromItem(self._table_widget.item(item.row(), item.column()+1))
        elif item.row() < self._table_widget.rowCount()-1:
            index = self._table_widget.indexFromItem(self._table_widget.item(item.row()+1, item.column()-1))
        else:
            index = self._table_widget.indexFromItem(self._table_widget.item(0, 0))

        self._table_widget.setCurrentIndex(index)
        self._table_widget.edit(index)
