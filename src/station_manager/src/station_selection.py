#!/usr/bin/python3
import sys
import os
import inspect
import signal
import time
import json
import copy
from typing import Callable, Dict
from websocket import create_connection
import autobahn.exception as aex

from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidgetItem, QDialog
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread, QObject
from PyQt5.QtGui import QIntValidator, QColor



from autobahn.twisted.websocket import WebSocketClientProtocol, WebSocketClientFactory

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

ERROR_DICT = {1 : "Erfolg", 2 : "Interner Server Fehler", 3 : "Keine Kapazität", 4 : "Station bereits belegt",
5 : "Station ausgeschalten", 6 : "Übung nicht Verfügbar", 7 :  "Gewicht nicht erkannt",
8 : "Anfrage nicht korrekt", 9 : "falsche id", 10 : "user bereits woanders angemeldet / falsche Station",
11 : "user ist in einer anderen Übung noch eingeloggt"}

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
        #self._app = QApplication(sys.argv)
        self._main_window = QMainWindow()
        self._main_window.closeEvent = self.closeEvent
        self._gui = StationSelectionUi()
        self._gui.setupUi(self._main_window)
        self.setupUi(self._main_window)
        self._is_connected = False
        self._data_manager : DataManager = data_manager
        self.activate_exercise_button.setEnabled(False)
        time.sleep(1)

        self.activate_station_button.clicked.connect(self.set_station_state)
        self.activate_exercise_button.clicked.connect(self.set_exercise_state)
        self.weight_detection_button.clicked.connect(self.start_weight_detection)

        signal.signal(signal.SIGINT, signal.SIG_DFL)

        only_int = QIntValidator()
        self.exercise_edit.setValidator(only_int)

        self.station_active = False
        self.exercise_active = False

        self._use_exercise_names = self._data_manager.is_mongo_on()
        if self._use_exercise_names:
            self.exercise_edit.setEnabled(False)
            self.load_exercises()
        else:
            self.exercise_combobox.setEnabled(False)

        self.error_text.setTextColor(QColor(235, 64, 52))
        self._send_message = None

    def client_callback(self, callback):
        #logy.debug("Register Message Callback", self._verbose)
        self._send_message = callback

    def show(self):
        stations = self._data_manager.get_station_names()

        for station_id, station_name in stations.items():
            self.station_combobox.addItem(station_name, station_id)
        self.station_combobox.addItem("video ./data/video.avi", 999)
        #self.station_combo.addItems(station_names)
        #self.video_combobox.addItems(names)
        self._main_window.show()
        #self._app.exec_()

    def load_exercises(self):
        exercises = self._data_manager.get_exercises()
        print(exercises)
        for exercise_id, exercise_name in exercises.items():
            self.exercise_combobox.addItem(exercise_name, exercise_id)

    def set_station_state(self):
        self.toggle_station()
        index = self.station_combobox.currentIndex()
        station_id = self.station_combobox.itemData(index, Qt.UserRole)
        data = copy.deepcopy(REQUEST_DICT)
        request = 1 if self.station_active else 2
        payload = {"station" : station_id, "exercise" : int(-1)}
        payload_str = json.dumps(payload)
        self.print_info("[REQUEST]")
        self.print_info(f"request={request}, payload={payload_str}")
        self._send_message(request, payload)

    def set_exercise_state(self):
        self.toggle_exercise()
        if self._use_exercise_names:
            index = self.exercise_combobox.currentIndex()
            exercise_id = self.exercise_combobox.itemData(index, Qt.UserRole)
        else:
            exercise_id = self.exercise_edit.text()
        data = copy.deepcopy(REQUEST_DICT)
        index = self.station_combobox.currentIndex()
        station_id = self.station_combobox.itemData(index, Qt.UserRole)
        request = 3 if self.exercise_active else 4
        payload = {"station" : station_id, "exercise" : int(exercise_id), "set_id" : 1}
        payload_str = json.dumps(payload)
        self.print_info("[REQUEST]")
        self.print_info(f"request={request}, payload={payload_str}")
        self._send_message(request, payload)

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
        #time.sleep(1000)
        index = self.station_combobox.currentIndex()
        station_id = self.station_combobox.itemData(index, Qt.UserRole)
        request = 7
        payload = {"station" : station_id, "time" : float(self.time_spin.value())}
        payload_str = json.dumps(payload)
        self.print_info("[REQUEST]")
        self.print_info(f"request={request}, payload={payload_str}")
        self._send_message(request, payload)

    def closeEvent(self, event):
        reactor.callFromThread(reactor.stop)
        print("LEAVE")

        # self.activate_exercise_button.setEnabled(False)
        # self.activate_station_button.setEnabled(False)
        # self._web_socket.send(json_str_exercise)
        # self.output_text.append(json_str_exercise)
        # id_data = self._web_socket.recv()
        # id_data = json.loads(id_data)
        # if id_data["response"] == 507:
        #     value = id_data["payload"]["weight"]
        #     self.weight_edit.setText(str(value))
        # self.activate_exercise_button.setEnabled(True)
        # self.activate_station_button.setEnabled(True)

    #def wait_for_message(self):

    def parse_message(self, msg : str):
        data = json.loads(msg)
        if "response" not in data or "type" not in data or "status_code" not in data:
            self.print_error(f"[PARSE ERROR] response, type or status_code not in message")
            self.print_error(msg)
            self.print_error("\n")

        if data["type"] != 1:
            self.print_error(f"[PARSE ERROR] type = {data['type']}. Must be 1")
            self.print_error(msg)
            self.print_error("\n")
            return {}

        if data["response"] == 508:
            error_string = ERROR_DICT[data["response"]]
            self.print_error(f"[SERVER ERROR] {error_string}")
            self.print_error(msg)
            self.print_error("\n")
            return {}

        self.print_info("[RESPONSE]")
        self.print_info(msg)
        self.print_info("\n")

    def print_info(self, msg : str):
        self.output_text.append(msg)

    def print_error(self, msg : str):
        self.error_text.append(msg)

    def login_station(self, msg : str):
        data = self.parse_message(msg)
        if not data:
            return

    def logout_station(self, msg : str):
        data = self.parse_message(msg)
        if not data:
            return

    def start_exercise(self, msg : str):
        data = self.parse_message(msg)
        if not data:
            return

    def stop_exercise(self, msg : str):
        data = self.parse_message(msg)
        if not data:
            return

    def get_weight_detection(self, msg : str):
        data = self.parse_message(msg)
        if not data:
            return

    def error_message(self, msg : str):
        data = self.parse_message(msg)
        if not data:
            return

    def get_repetition(self, msg : str):
        data = self.parse_message(msg)
        if not data:
            return

class MyClientProtocol(WebSocketClientProtocol):

    def onConnect(self, response):
        if self.factory._register_client_callback is not None:
            self.factory._register_client_callback(self.send_msg_ts)
        print("Server connected: {0}".format(response.peer))

    def send_msg_ts(self, request_code=0, payload=dict({})):
        reactor.callFromThread(self.send_msg, request_code, payload)

    def send_msg(self, request_code=0, payload=dict({})):
        response = copy.deepcopy(REQUEST_DICT)
        response["id"] = self._id
        response["request"] = request_code
        response["payload"] = payload
        response = str(json.dumps(response))
        response = response.encode('utf8')
        #print("[REQUEST]:", response)
        try:
            self.sendMessage(response, False)
        except aex.Disconnected:
            print("[DISCONNECTED]: Could not send response")
        except Exception:
            print("[UNKNOWN ERROR]: Could not send response")

    def onMessage(self, payload, isBinary):
        if isBinary:
            print("[ERROR] Can not handle binary")
            return

        msg = payload.decode('utf8')
        #print("[RESPONSE]", msg)
        data = json.loads(msg)
        if "response" not in data:
            print("[ERROR] No response field")

        if int(data["response"]) == 500:
            if "id" not in data:
                print("[ERROR] No id field")
                return
            self._id = data["id"]
            return

        request_func = self.factory._callbacks.get(int(data["response"]))
        if request_func is None:
            print("Request currently not implemented", 2)
            return

        request_func(msg)

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))

class ClientController(WebSocketClientFactory):

    def __init__(self, uri, register_client_callback = None):
        print("Init Server Factory")
        WebSocketClientFactory.__init__(self, uri)
        self._callbacks : Dict[int, Callable] = {}
        self._register_client_callback = register_client_callback

    def register_callback(self, message_code : int, callback : Callable):
        self._callbacks[message_code] = callback

if __name__=="__main__":
    app = QApplication(sys.argv)
    import qt5reactor
    qt5reactor.install()
    from twisted.internet import reactor

    data_manager_ = DataManager()
    station_selection = StationSelection(data_manager_)
    factory = ClientController("ws://127.0.0.1:3030", station_selection.client_callback)
    factory.protocol = MyClientProtocol
    factory.register_callback(501, station_selection.login_station)
    factory.register_callback(502, station_selection.logout_station)
    factory.register_callback(503, station_selection.start_exercise)
    factory.register_callback(504, station_selection.stop_exercise)
    factory.register_callback(507, station_selection.get_weight_detection)
    factory.register_callback(508, station_selection.error_message)
    factory.register_callback(509, station_selection.get_repetition)
    station_selection.show()
    print("SHOW END")
    #app.exec_()
    print("EXEC END")
    reactor.connectTCP("127.0.0.1", 3030, factory)
    reactor.run()
