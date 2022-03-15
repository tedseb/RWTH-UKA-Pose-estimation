import copy
from msilib.schema import Error
import sys
import traceback
import threading
from typing import Callable, Dict
from multiprocessing import Lock
import json
import time
import bluetooth
from ..station_manager_response import SMResponse
import logy

RESPONSE_DICT = {
    "id" : "",
    "type" : 1,
    "response": 508,
    "status_code": 0,
    "payload": {}
}

class BtServerSocket:
    _err_to_str = {
        1 : "Success",
        2 : "Internal Server Error",
        3 : "No Capaticity",
        4 : "Station is already in use",
        5 : "Station is offline",
        6 : "Exercise not available",
        7 : "Can not detect weight",
        8 : "Error in Request",
        9 : "Wrong user ID",
        10 : "User already loged in into another station or station does not exist",
        11 : "User started already an exercise. This should be finished first.",
    }

    def __init__(self, factory) -> None:
        self._factory = factory

    def onConnect(self, internal_id : int):
        print("Client connecting")
        self._id = internal_id
        if self._factory._register_client_callback is not None:
            self._factory._register_client_callback(self._id, self.send_msg_ts)

        self._factory._logger.info("New client connection")
        response = copy.deepcopy(RESPONSE_DICT)
        response["id"] = self._id
        response["response"] = 500
        response["status_code"] = 1
        response = json.dumps(response).encode('utf8')
        self._send_msg(response, False)

    def callback_wrapper(self, function : Callable, pyaload : Dict):
        #pylint: disable=broad-except
        try :
            result : SMResponse = function(self._id, pyaload)
            if result.status_code != 1:
                logy.warn(f"Client Connection Status {result.status_code}: {self._err_to_str[result.status_code]}")
        except Exception as exception:
            self.send_error_ts(str(exception))
            trace = traceback.format_exc()
            self._factory._logger.error(f"Could not handle client request: \n {trace}")
            return

        #traceback.print_exc()
        if not result.request_requiered:
            return

        self.send_msg_ts(result.response_code, result.status_code, result.payload)

    def onMessage(self, payload, isBinary):
        if isBinary:
            self.send_error("Binary Messages currently not supported", 2)
            return

        data_str = str(payload.decode('utf8'))
        try:
            data = json.loads(data_str)
        except json.decoder.JSONDecodeError:
                self.send_error("Wrong JSON Format", 8)
                return

        #self._factory._logger.debug("Received Data ", str(data))
        #user_id = data.get("id")
        # if user_id is None:
        #     self.send_error("There is no 'id' field in the request", 8)
        #     return
        # if user_id != self._id:
        #     self.send_error("Wrong ID", 8)
        #     return

        message_type = data.get("type")
        if message_type is None:
            self.send_error("There is no 'type' field in the request", 8)
            return
        if message_type != 0:
            self.send_error("Message is no Request Type", 8)
            return

        request = data.get("request")
        if request is None:
            self.send_error("There is no 'request' field in the request", 8)
            return
        if request < 1 or request > 499:
            self.send_error("reqeuest code range must be in the range from 1 to 499", 8)
            return

        payload = data.get("payload")
        if payload is None:
            self.send_error("There is no 'payload' field in the request", 8)
            return

        request_func = self._factory._callbacks.get(request)
        if request_func is None:
            self.send_error("Request currently not implemented", 2)
            return

        reactor.callInThread(self.callback_wrapper, request_func, payload)
        return

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))

    def send_error_ts(self, error="Error", satus_code=2, response_code=508):
        reactor.callFromThread(self.send_error, error, satus_code, response_code)

    def send_error(self, error="Error", satus_code=2, response_code=508):
        self.send_msg(response_code, satus_code, {"error" : error})

    def send_msg_ts(self, response_code=508, satus_code=2, payload=dict({})):
        reactor.callFromThread(self.send_msg, response_code, satus_code, payload)

    def send_msg(self, response_code=508, satus_code=2, payload=dict({})):
        response = copy.deepcopy(RESPONSE_DICT)
        response["id"] = self._id
        response["response"] = response_code
        response["status_code"] = satus_code
        response["payload"] = payload
        response = str(json.dumps(response))
        response = response.encode('utf8')
        print("[RESPONSE]:", response)
        try:
            self._send_msg(response, False)
        # except aex.Disconnected:
        #     print("[DISCONNECTED]: Could not send response")
        except Exception:
            print("[UNKNOWN ERROR]: Could not send response")

    def _send_msg(self):
        pass

class BtServerController:

    def __init__(self, uri, register_client_callback = None):
        print("Init Server Factory")
        #WebSocketServerFactory.__init__(self, uri)

        self._callbacks : Dict[int, Callable] = {}
        self._register_client_callback = register_client_callback
        self._logger = logy.get_or_create_logger("Server", logy.DEBUG, "SERVER")
        self._messages = []
        self._messages_mutex = Lock()

    def register_callback(self, message_code : int, callback : Callable):
        self._callbacks[message_code] = callback

    def add_msg(self, user_id, msg):
        with self._messages_mutex:
            self._messages.append((user_id, msg))

    def get_and_clear_msgs(self):
        with self._messages_mutex:
            messages_copy = self._messages
            self._messages = []
        return messages_copy

class BtServerListener:
    def __init__(self, server_controller : BtServerController):
        self._clients = {}
        self._new_connections = []
        self._data_mutex = Lock()
        self._server_controller = server_controller
        self._advertise_thread = threading.Thread(target=self.advertise_service)
        self._detection_thread.start()
        self._is_active = True
        self._wait_time = 0.025
        self._ids = 0

    def get_id(self) -> str:
        self._ids += 1
        return f"user_{self._ids}"

    def delete_user_client(self, user_id):
        self._server_controller._messages.pop(user_id, None)
        self._clients.pop(user_id, None)

    def start_update_loop(self):
        while self._is_active:
            start_time = time.time()
            self.create_new_clients()
            self.check_valid_connections()
            self.process_new_received_data()
            self.process_new_msgs()
            wait_time = self._wait_time - (time.time() - start_time)
            if wait_time > 0:
                time.sleep(wait_time)

    def create_new_clients(self):
        while self._new_connections:
            client = self._new_connections.pop(0)
            server_connection = BtServerSocket(self._server_controller)
            client_id = self.get_id()
            self._clients[client_id] = (client, server_connection)
            server_connection.onConnect(client_id)

    def check_valid_connections(self):
        deletion_list = []
        for client_id, (client, server_connection) in self._clients.items():
            try:
                client.getpeername()
            except:
                self._server_controller._logger.info(f"Client {client_id} disconnected from BT Server")
                client.close()
                #del server_connection
                deletion_list.append(client_id)

        for client_id in deletion_list:
            del self._clients[client_id]

    def process_new_received_data(self):
        for (client, server_connection) in self._clients.values():
            try:
                data = client.recv(1024)
                server_connection.onMessage(data, False)
            except InterruptedError:
                pass
            except Exception as e:
                self._server_controller._logger(e)

    def process_new_msgs(self):
        messages = self._server_controller.get_and_clear_msgs()
        for (user_id, message) in messages:
            self._clients[user_id].send(message)

    def advertise_service(self):
        user_id = self.get_id()
        server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        server_sock.bind(("", bluetooth.PORT_ANY))
        server_sock.listen(8)
        port = server_sock.getsockname()[1]
        uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
        self._server_controller._logger.debug(f"Waiting for connection on RFCOMM channel: {port}")
        bluetooth.advertise_service(server_sock, "SampleServer", service_id=uuid,
                                    service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                                    profiles=[bluetooth.SERIAL_PORT_PROFILE],
                                    # protocols=[bluetooth.OBEX_UUID]
                                    )
        while self._is_active:
            client_sock, client_info = server_sock.accept()
            client_sock.settimeout(0)
            self._server_controller._logger.debug(f"Accepted connection from {client_info}")
            self._new_connections.append(client_sock)
            self._server_controller._messages[user_id] = []
