import copy
import sys
import traceback
from typing import Callable, Dict
import json
import autobahn.exception as aex
from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory
from twisted.internet import reactor
from ..station_manager_response import SMResponse
import logy

RESPONSE_DICT = {
    "id" : "",
    "type" : 1,
    "response": 508,
    "status_code": 0,
    "payload": {}
}

class ServerSocket(WebSocketServerProtocol):
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

    def onConnect(self, request):
        print("Client connecting: {0}".format(request.peer))
        self._id = self.factory.get_id()
        if self.factory._register_client_callback is not None:
            self.factory._register_client_callback(self._id, self.send_msg_ts)

    def onOpen(self):
        self.factory._logger.info("New client connection")
        response = copy.deepcopy(RESPONSE_DICT)
        response["id"] = self._id
        response["response"] = 500
        response["status_code"] = 1
        response = json.dumps(response).encode('utf8')
        self.sendMessage(response, False)

    def callback_wrapper(self, function : Callable, pyaload : Dict):
        #pylint: disable=broad-except
        try :
            result : SMResponse = function(self._id, pyaload)
            if result.status_code != 1:
                logy.warn(f"Client Connection Status {result.status_code}: {self._err_to_str[result.status_code]}")
        except Exception as exception:
            self.send_error_ts(str(exception))
            trace = traceback.format_exc()
            self.factory._logger.error(f"Could not handle client request: \n {trace}")
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
        logy.warn(data_str)
        try:
            data = json.loads(data_str)
        except json.decoder.JSONDecodeError:
                self.send_error("Wrong JSON Format", 8)
                return

        logy.warn(str(data))
        logy.warn(type(data))
        #self.factory._logger.debug("Received Data ", str(data))
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

        request_func = self.factory._callbacks.get(request)
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
            self.sendMessage(response, False)
        except aex.Disconnected:
            print("[DISCONNECTED]: Could not send response")
        except Exception:
            print("[UNKNOWN ERROR]: Could not send response")

class ServerController(WebSocketServerFactory):

    def __init__(self, uri, register_client_callback = None):
        print("Init Server Factory")
        WebSocketServerFactory.__init__(self, uri)
        self._ids = 0
        self._callbacks : Dict[int, Callable] = {}
        self._register_client_callback = register_client_callback
        self._logger = logy.get_or_create_logger("Server", logy.DEBUG, "SERVER")

    def register_callback(self, message_code : int, callback : Callable):
        self._callbacks[message_code] = callback

    def get_id(self) -> str:
        self._ids += 1
        return f"user_{self._ids}"
