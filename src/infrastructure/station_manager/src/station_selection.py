import sys
import signal
from typing import Callable
from websocket import create_connection
from src.layouts import StationSelectionUi
from .data_manager import DataManager
from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidgetItem, QDialog
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread, QObject

MOBILE_SERVER = "ws://localhost:3030/"

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
        try:
            self._web_socket = create_connection(MOBILE_SERVER)
            self._is_connected = True
        except ConnectionRefusedError:
            print(f"Could not connect to {MOBILE_SERVER}")
        self.confirm_button.clicked.connect(self.station_confirm)

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
        data = self.station_combobox.itemData(index, Qt.UserRole)
        is_active = "true" if self.active_box.isChecked() else "false"
        exercise = self.exercise_edit.text()
        json_str = "{ \n" + \
            f'   "stationID" : {data}, \n' + \
            f'   "isActive" : {is_active}, \n' + \
            f'   "exerciseName" : "{exercise}" \n' + \
            "} \n"
        self.output_text.append(json_str)

        if not self._is_connected:
            try:
                self._web_socket = create_connection(MOBILE_SERVER)
                self._is_connected = True
            except ConnectionRefusedError:
                print(f"Could not connect to {MOBILE_SERVER}")

        if self._is_connected:
            self._web_socket.send(json_str)

if __name__=="__main__":
    a = StationSelection()
    a.show()
