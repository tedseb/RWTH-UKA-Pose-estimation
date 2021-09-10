#!/usr/bin/python3
import sys
import os
import inspect
import signal
import time
import json
import copy
from typing import Callable
from websocket import create_connection
from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidgetItem, QDialog
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread, QObject

if __name__=="__main__":
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    sys.path.insert(0, parentdir)

from src.layouts import StationSelectionUi
from src.data_manager import DataManager

MOBILE_SERVER = "ws://localhost:9000/"

REQUEST_DICT = {
    "id" : "",
    "type" : 0,
    "request": 1,
    "payload": {}
}

class StationSelection(StationSelectionUi, QObject):
    def __init__(self, data_manager = None):
        super().__init__()
        if data_manager is None:
            raise RuntimeError('This Gui Needs a data manager')
        self._app = QApplication(sys.argv)
        self._main_window = QMainWindow()
        self._gui = StationSelectionUi()
        self._gui.setupUi(self._main_window)
        self.setupUi(self._main_window)
        self._is_connected = False
        self._data_manager : DataManager = data_manager
        time.sleep(1)
        try:
            self._web_socket = create_connection(MOBILE_SERVER)
            self._is_connected = True
        except ConnectionRefusedError:
            print(f"Could not connect to {MOBILE_SERVER}")
        self.confirm_button.clicked.connect(self.station_confirm)
        id_data = self._web_socket.recv()
        id_data = json.loads(id_data)
        self._id = id_data["id"]
        signal.signal(signal.SIGINT, signal.SIG_DFL)

    def show(self):
        stations = self._data_manager.get_station_names()
        for station_id, station_name in stations.items():
            self.station_combobox.addItem(station_name, station_id)
        #self.station_combo.addItems(station_names)
        #self.video_combobox.addItems(names)
        self._main_window.show()
        sys.exit(self._app.exec_())

    def station_confirm(self):
        index = self.station_combobox.currentIndex()
        station_id = self.station_combobox.itemData(index, Qt.UserRole)
        is_active = self.active_box.isChecked()
        exercise = self.exercise_edit.text()
        data = copy.deepcopy(REQUEST_DICT)
        data["id"] = self._id
        data["request"] = 1 if is_active else 2
        data["payload"] = {"station" : station_id, "exercise" : exercise}
        json_str = json.dumps(data)
        self.output_text.append(json_str)

        if not self._is_connected:
            try:
                self._web_socket = create_connection(MOBILE_SERVER)
                self._is_connected = True
            except ConnectionRefusedError:
                print(f"Could not connect to {MOBILE_SERVER}")

        if self._is_connected:
            self._web_socket.send(json_str)
            print(self._web_socket.recv())

if __name__=="__main__":
    data_manager = DataManager()
    a = StationSelection(data_manager)
    a.show()
