import copy
import sys
import traceback
from typing import Callable, Dict
import json
import autobahn.exception as aex
from .abstract_server import ServerController, ServerSocket
from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory
from twisted.internet import reactor
from ..station_manager_response import SMResponse
import logy

class WebServerSocket(WebSocketServerProtocol, ServerSocket):

    def onConnect(self, request):
        self.init_socket(self.factory, self.send_msg, self.factory._logger)

    def onOpen(self):
        self._on_connection()

    def onMessage(self, payload, is_binary):
        self._on_message(payload, is_binary)

    def onClose(self, wasClean, code, reason):
        self._on_close()

    def send_msg(self, msg, is_binary):
        reactor.callFromThread(self.sendMessage, msg, False)

    def start_new_thread(self, function, *argparams):
        self._logger.info("reactor")
        reactor.callInThread(function, *argparams)

class WebServerController(WebSocketServerFactory, ServerController):

    def __init__(self, uri, register_client_callback = None):
        #print("Init Server Factory")
        logger = logy.get_or_create_logger("WebServer", logy.DEBUG, "WEBSOCK")
        WebSocketServerFactory.__init__(self, uri)
        ServerController.__init__(self, logger)
