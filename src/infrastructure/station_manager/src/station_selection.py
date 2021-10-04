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
from PyQt5.QtGui import QIntValidator

if __name__=="__main__":
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    sys.path.insert(0, parentdir)

from src.layouts import StationSelectionUi
from src.data_manager import DataManager

MOBILE_SERVER = "ws://localhost:3030/"

REQUEST_DICT = {
    "id" : "",
    "type" : 0,
    "request": 1,
    "payload": {}
}

def make_items_from_dict(labels):
    item_list = []
    for key, value in labels.items():
        item = QListWidgetItem()
        item.setText(value)
        item.setData(Qt.UserRole, int(key))
        item_list.append(item)
    return item_list

def insert_items_into_widget(qt_widget, item_list):
    for i in item_list:
        qt_widget.addItem(i)

def insert_dict_into_widget(qt_widget, label_dict):
    item_list = make_items_from_dict(label_dict)
    insert_items_into_widget(qt_widget, item_list)

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
        self.activate_exercise_button.setEnabled(False)
        time.sleep(1)
        try:
            self._web_socket = create_connection(MOBILE_SERVER)
            self._is_connected = True
        except ConnectionRefusedError:
            print(f"Could not connect to {MOBILE_SERVER}")
        self.activate_station_button.clicked.connect(self.set_station_state)
        self.activate_exercise_button.clicked.connect(self.set_exercise_state)
        self.weight_detection_button.clicked.connect(self.start_weight_detection)
        id_data = self._web_socket.recv()
        id_data = json.loads(id_data)
        self._id = id_data["id"]
        signal.signal(signal.SIGINT, signal.SIG_DFL)

        only_int = QIntValidator()
        self.exercise_edit.setValidator(only_int)

        self.station_active = False
        self.exercise_active = False

        self._use_exercise_names = self._data_manager._mongo_is_on()
        if self._use_exercise_names:
            self.exercise_edit.setEnabled(False)
        else:
            self.exercise_combobox.setEnabled(False)
            self.load_exercises()

    def show(self):
        stations = self._data_manager.get_station_names()
        for station_id, station_name in stations.items():
            self.station_combobox.addItem(station_name, station_id)
        #self.station_combo.addItems(station_names)
        #self.video_combobox.addItems(names)
        self._main_window.show()
        sys.exit(self._app.exec_())

    def load_exercises(self):
        exercises = self._data_manager.get_exercises()
        for exercise_id, exercise_name in exercises.items():
            self.station_combobox.addItem(exercise_name, exercise_id)

    def set_station_state(self):
        self.toggle_station()
        index = self.station_combobox.currentIndex()
        station_id = self.station_combobox.itemData(index, Qt.UserRole)
        data = copy.deepcopy(REQUEST_DICT)
        data["id"] = self._id
        data["request"] = 1 if self.station_active else 2
        data["payload"] = {"station" : station_id, "exercise" : int(-1)}
        json_str_station = json.dumps(data)
        
        if not self._is_connected:
            try:
                self._web_socket = create_connection(MOBILE_SERVER)
                self._is_connected = True
            except ConnectionRefusedError:
                print(f"Could not connect to {MOBILE_SERVER}")

        if self._is_connected:
            self._web_socket.send(json_str_station)
            self.output_text.append(json_str_station)
            print(self._web_socket.recv())

    def set_exercise_state(self):
        self.toggle_exercise()
        exercise = self.exercise_edit.text()
        data = copy.deepcopy(REQUEST_DICT)
        index = self.station_combobox.currentIndex()
        station_id = self.station_combobox.itemData(index, Qt.UserRole)
        data["id"] = self._id        
        data["request"] = 3 if self.exercise_active else 4
        data["payload"] = {"station" : station_id, "exercise" : int(exercise)}
        data["payload"]["set_id"] = 1
        json_str_exercise = json.dumps(data)
        
        if self._is_connected:
            self._web_socket.send(json_str_exercise)
            self.output_text.append(json_str_exercise)
            print(self._web_socket.recv())

    def toggle_exercise(self): 
        if self.exercise_active:
            self.exercise_edit.setEnabled(True)
            self.activate_exercise_button.setText("Start Exercise")
            self.activate_station_button.setEnabled(True)
            self.weight_detection_button.setEnabled(True)
        else:
            self.exercise_edit.setEnabled(False)
            self.activate_exercise_button.setText("Stop Exercise")
            self.activate_station_button.setEnabled(False)
            self.weight_detection_button.setEnabled(False)

        self.exercise_active = not self.exercise_active

    def toggle_station(self): 
        if self.station_active:
            self.station_combobox.setEnabled(True)
            self.activate_station_button.setText("Activate Station")
            self.activate_exercise_button.setEnabled(False)
            self.weight_detection_button.setEnabled(False)
        else:
            self.station_combobox.setEnabled(False)
            self.activate_station_button.setText("Deactivate Station")
            self.activate_exercise_button.setEnabled(True)
            self.weight_detection_button.setEnabled(True)

        self.station_active = not self.station_active

    def start_weight_detection(self):
        index = self.station_combobox.currentIndex()
        station_id = self.station_combobox.itemData(index, Qt.UserRole)
        data = copy.deepcopy(REQUEST_DICT)
        data["id"] = self._id        
        data["request"] = 7
        data["payload"] = {"station" : station_id, "time" : float(self.time_spin.value())}
        json_str_exercise = json.dumps(data)
        
        if self._is_connected:
            self.activate_exercise_button.setEnabled(False)
            self.activate_station_button.setEnabled(False)
            self._web_socket.send(json_str_exercise)
            self.output_text.append(json_str_exercise)
            id_data = self._web_socket.recv()
            id_data = json.loads(id_data)
            if id_data["response"] == 507:
                value = id_data["payload"]["weight"]
                self.weight_edit.setText(str(value))
            self.activate_exercise_button.setEnabled(True)
            self.activate_station_button.setEnabled(True)

if __name__=="__main__":
    data_manager = DataManager()
    a = StationSelection(data_manager)
    a.show()
