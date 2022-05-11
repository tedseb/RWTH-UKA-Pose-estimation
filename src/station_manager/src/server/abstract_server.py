import threading
import json
import copy
from typing import Callable, Dict
import traceback
from abc import ABC, abstractmethod
from ..station_manager_response import SMResponse
import logy

RESPONSE_DICT = {
    "id" : "",
    "type" : 1,
    "response": 508,
    "status_code": 0,
    "payload": {}
}

class ServerController(ABC):
    _ids = 0
    def __init__(self, logger_name="Server"):
        self._ids = 0
        self._callbacks : Dict[int, Callable] = {}
        self._register_client_callback : Callable[[str, Callable], None] = None
        self._logger = logy.get_or_create_logger(logger_name, logy.DEBUG, "SERVER")

    @abstractmethod
    def run(self):
        raise NotImplementedError

    @abstractmethod
    def kill(self):
        raise NotImplementedError

    def register_callback(self, message_code : int, callback : Callable):
        self._callbacks[message_code] = callback

    def set_register_client_callback(self, callback : Callable[[str, Callable], None]):
        self._register_client_callback = callback

    @staticmethod
    def get_id() -> str:
        ServerController._ids += 1
        return f"user_{ServerController._ids}"

class ServerSocket(ABC):
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
        12 : "User not loged into a station/ecercise",
        12 : "User not loged into a station/ecercise"
    }

    def __init__(self) -> None:
        self._factory = None
        self._logger = None
        self._id = self._factory.get_id()
        self._send_message = None

    def init_socket(self, factory : ServerController, send_message : Callable[[bytes, bool], None], logger):
        self._factory = factory
        self._logger = logger
        self._id = self._factory.get_id()
        self._send_message = send_message
        self._logger.info(f"Init Socket")

    def _on_connection(self, request = None):
        if self._factory._register_client_callback is not None:
            self._factory._register_client_callback(self._id, self._send_msg)
        self._logger.info(f"New client connection {self._id}")
        response = copy.deepcopy(RESPONSE_DICT)
        response["id"] = self._id
        response["response"] = 500
        response["status_code"] = 1
        response = json.dumps(response)
        response = response.encode('utf8')
        self._send_message(response, False)

    def _callback_wrapper(self, function : Callable, pyaload : Dict):
        #pylint: disable=broad-except
        self._logger.info("Thread Started")
        try :
            result : SMResponse = function(self._id, pyaload)
            if result.status_code != 1:
                logy.warn(f"Client Connection Status {result.status_code}: {self._err_to_str[result.status_code]}")
        except Exception as exception:
            self.send_error_ts(str(exception))
            trace = traceback.format_exc()
            self._logger.error(f"Could not handle client request: \n {trace}")
            return

        #traceback.print_exc()
        if not result.request_requiered:
            return

        self._send_msg(result.response_code, result.status_code, result.payload)

    def _on_message(self, payload, isBinary):
        if isBinary:
            self._send_error("Binary Messages currently not supported", 2)
            return
        self._logger.info(f"New Message={payload}")

        data_str = str(payload.decode('utf8'))
        #logy.warn(data_str)
        try:
            data = json.loads(data_str)
        except json.decoder.JSONDecodeError:
                self._send_error("Wrong JSON Format", 8)
                return

        message_type = data.get("type")
        if message_type is None:
            self._send_error("There is no 'type' field in the request", 8)
            return
        if message_type != 0:
            self._send_error("Message is no Request Type", 8)
            return

        request = data.get("request")
        if request is None:
            self._send_error("There is no 'request' field in the request", 8)
            return
        if request < 1 or request > 499:
            self._send_error("reqeuest code range must be in the range from 1 to 499", 8)
            return

        payload = data.get("payload")
        if payload is None:
            self._send_error("There is no 'payload' field in the request", 8)
            return

        request_func = self._factory._callbacks.get(request)
        if request_func is None:
            self._send_error("Request currently not implemented", 2)
            return

        self._logger.info("New Message, start Thread")
        self.start_new_thread(self._callback_wrapper, request_func, payload)
        #threading.Thread(target=self._callback_wrapper, args=(request_func, payload,), daemon=True)
        return

    def start_new_thread(self, function, *argparams):
        self._logger.info("not reactor")
        thread = threading.Thread(target=function, args=argparams, daemon=True)
        thread.start()

    def _on_close(self):
        self._logger.debug("WebSocket connection closed")

    def _send_error(self, error="Error", satus_code=2, response_code=508):
        self._send_msg(response_code, satus_code, {"error" : error})

    def _send_msg(self, response_code=508, satus_code=2, payload=dict({})):
        response = copy.deepcopy(RESPONSE_DICT)
        response["id"] = self._id
        response["response"] = response_code
        response["status_code"] = satus_code
        response["payload"] = payload
        response = str(json.dumps(response))
        response = response.encode('utf8')
        #print("[RESPONSE]:", response)
        try:
            self._send_message(response, False)
        except Exception:
            self._logger.error("")
