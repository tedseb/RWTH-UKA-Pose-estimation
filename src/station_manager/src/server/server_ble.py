#!/usr/bin/python3
# Advertises, accepts a connection, creates the microbit temperature service, LED service and their characteristics

from gzip import READ
from xmlrpc.client import boolean
from . import bluetooth_constants as bluetooth_constants
from . import bluetooth_gatt as bluetooth_gatt
from . import bluetooth_utils as bluetooth_utils
from . import bluetooth_exceptions as bluetooth_exceptions
import dbus
import dbus.exceptions
import dbus.service
import dbus.mainloop.glib
import sys
import traceback
import random
from typing import Callable, Dict, List
import copy
import logy
import threading
from gi.repository import GObject
from gi.repository import GLib
import json
from ..station_manager_response import SMResponse
sys.path.insert(0, '.')

UUIDS = ["ac00", "ac01", "ac02", "ac03"]
# much of this code was copied or inspired by test\example-advertisement in the BlueZ source

RESPONSE_DICT = {
    "id" : "",
    "type" : 1,
    "response": 508,
    "status_code": 0,
    "payload": {}
}

class BleServerSocket:
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
        12 : "User not loged into a station/ecercise"
    }

    def __init__(self):
        self._factory = None
        self._id = ""
        self.sendMessage = None

    def onConnect(self, request = None):
        self._id = self._factory.get_id()
        if self._factory._register_client_callback is not None:
            self._factory._register_client_callback(self._id, self.send_msg)
        self._factory.logger.info(f"New client connection {self._id}")
        response = copy.deepcopy(RESPONSE_DICT)
        response["id"] = self._id
        response["response"] = 500
        response["status_code"] = 1
        response = json.dumps(response)
        self.sendMessage(response, False)

    def callback_wrapper(self, function : Callable, pyaload : Dict):
        #pylint: disable=broad-except
        self._factory.logger.info("Thread Started")
        try :
            result : SMResponse = function(self._id, pyaload)
            if result.status_code != 1:
                logy.warn(f"Client Connection Status {result.status_code}: {self._err_to_str[result.status_code]}")
        except Exception as exception:
            self.send_error(str(exception))
            trace = traceback.format_exc()
            self._factory.logger.error(f"Could not handle client request: \n {trace}")
            return

        #traceback.print_exc()
        if not result.request_requiered:
            return

        self.send_msg(result.response_code, result.status_code, result.payload)

    @logy.catch_thread
    def onMessage(self, payload, isBinary):
        if isBinary:
            self.send_error("Binary Messages currently not supported", 2)
            return
        self._factory.logger.info(f"New Message={payload}")

        data_str = str(payload)
        #logy.warn(data_str)
        try:
            data = json.loads(data_str)
        except json.decoder.JSONDecodeError:
                self.send_error("Wrong JSON Format", 8)
                return

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

        self._factory.logger.info("New Message, start Thread")
        thread = threading.Thread(target=self.callback_wrapper, args=(request_func, payload,), daemon=True)
        thread.start()
        return

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))

    def send_error(self, error="Error", satus_code=2, response_code=508):
        self._factory.logger.error(f"error: satus_code {satus_code}, response_code {response_code}")
        self.send_msg(response_code, satus_code, {"error" : error})

    def send_msg(self, response_code=508, satus_code=2, payload=dict({})):
        response = copy.deepcopy(RESPONSE_DICT)
        response["id"] = self._id
        response["response"] = response_code
        response["status_code"] = satus_code
        response["payload"] = payload
        response = str(json.dumps(response))
        #response = response.encode('utf8')
        print("[RESPONSE]:", response)
        try:
            self.sendMessage(response, False)
        except Exception:
            print("[UNKNOWN ERROR]: Could not send response")

class Advertisement(dbus.service.Object):
    PATH_BASE = '/org/bluez/ldsg/advertisement'

    def __init__(self, bus, index, advertising_type):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.ad_type = advertising_type
        self.service_uuids = None
        self.manufacturer_data = None
        self.solicit_uuids = None
        self.service_data = None #{service_uuids[0]: dbus.Array([0x01, 0x04, 0x01, 0x04], signature='y')}
        self.local_name = 'Hello'
        self.include_tx_power = False
        self.data = None #{0x26: dbus.Array([0x01, 0x01, 0x01], signature='y')}
        self.discoverable = True
        self.logger = logy.get_or_create_logger("BleServer", logy.DEBUG, "BLE")
        dbus.service.Object.__init__(self, bus, self.path)

    def set_service_uuid(self, uuid : str):
        complete_uuid = uuid
        if len(uuid) == 4:
            uuid_short = uuid.zfill(8)
            complete_uuid = f"{uuid_short}-0000-1000-8000-00805f9b34fb"
        self.service_uuids = [complete_uuid]

    def get_properties(self):
        properties = dict()
        properties['Type'] = self.ad_type
        if self.service_uuids is not None:
            properties['ServiceUUIDs'] = dbus.Array(self.service_uuids,
                                                    signature='s')
        if self.solicit_uuids is not None:
            properties['SolicitUUIDs'] = dbus.Array(self.solicit_uuids,
                                                    signature='s')
        if self.manufacturer_data is not None:
            properties['ManufacturerData'] = dbus.Dictionary(
                self.manufacturer_data, signature='qv')
        if self.service_data is not None:
            properties['ServiceData'] = dbus.Dictionary(self.service_data,
                                                        signature='sv')
        if self.local_name is not None:
            properties['LocalName'] = dbus.String(self.local_name)
        if self.discoverable is not None and self.discoverable == True:
            properties['Discoverable'] = dbus.Boolean(self.discoverable)
        if self.include_tx_power:
            properties['Includes'] = dbus.Array(["tx-power"], signature='s')

        if self.data is not None:
            properties['Data'] = dbus.Dictionary(
                self.data, signature='yv')
        self.logger.info(str(properties))
        return {bluetooth_constants.ADVERTISING_MANAGER_INTERFACE: properties}

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(bluetooth_constants.DBUS_PROPERTIES,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        self.logger.info(f"{interface} try to get props")
        if interface != bluetooth_constants.ADVERTISEMENT_INTERFACE:
            raise bluetooth_exceptions.InvalidArgsException()
        return self.get_properties()[bluetooth_constants.ADVERTISING_MANAGER_INTERFACE]

    @dbus.service.method(bluetooth_constants.ADVERTISING_MANAGER_INTERFACE,
                         in_signature='',
                         out_signature='')
    def Release(self):
        print('%s: Released' % self.path)


class BleServerService(bluetooth_gatt.Service):
#   Fake micro:bit LED service that uses the console as a pretend microbit LED display for text only
#   ref: https://lancaster-university.github.io/microbit-docs/resources/bluetooth/bluetooth_profile.html
#   LED Matrix characteristic not implemented to keep things simple

     def __init__(self, bus, path_base, index, uuid):
        bluetooth_gatt.Service.__init__(self, bus, path_base, index, f"0000{uuid}-0000-1000-8000-00805f9b34fb", True)
        characteristic_uuid = int(uuid, 16) + 100
        characteristic_uuid = f"{characteristic_uuid:04x}"
        self._characteristic = BleServerCharacteristic(bus, 0, self, characteristic_uuid)
        self.add_characteristic(self._characteristic)

class BleServerCharacteristic(bluetooth_gatt.Characteristic):

    def __init__(self, bus, index, service, uuid):
        bluetooth_gatt.Characteristic.__init__(
                self, bus, index,
                uuid,
                ['write', 'read', 'notify'],
                service)
        self.notifying = False
        self._on_msg = None
        self._on_connect = None
        self.logger = logy.get_or_create_logger("BleServer", logy.DEBUG, "BLE")
        self.logger.info(f"Adding Characteristic uuid {uuid}")

    def WriteValue(self, value, options):
        ascii_bytes = bluetooth_utils.dbus_to_python(value)
        text = ''.join(chr(i) for i in ascii_bytes)
        self.logger.info("Write")
        if self._on_msg is not None:
            self.logger.info("Write callback")
            self._on_msg(text, False)

    def ReadValue(self, options):
        self.logger.info('ReadValue in BleServer called')
        value = []
        value.append(dbus.int32(10))
        return value

    def send_msg(self, text, is_byte) -> bool:
        self.logger.info(f"send_msg in characteristics: {text}")
        if is_byte:
            self.logger.error(f"is byte")
            #return False

        if not self.notifying:
            self.logger.error(f"not self.notifying")

            #return False

        self.logger.info(f"try notifying {text} / {type(text)}")
        values = text.encode("utf-8")
        value = []
        for byte in values:
            value.append(dbus.Byte(byte))

        self.logger.info(f"notifying")
        self.PropertiesChanged(bluetooth_constants.GATT_CHARACTERISTIC_INTERFACE, { 'Value': value }, [])
        return True

    def StartNotify(self):
        self.logger.info("starting notifications")
        self.notifying = True
        if self._on_msg is not None:
            self._on_connect()

    def StopNotify(self):
        self.logger.info("stopping notifications")
        self.notifying = False


class Application(dbus.service.Object):
    """
    org.bluez.GattApplication1 interface implementation
    """
    def __init__(self, bus, p_services : List[BleServerService]):
        self.path = '/'
        self.services = []
        self.logger = logy.get_or_create_logger("BleServer", logy.DEBUG, "BLE")

        dbus.service.Object.__init__(self, bus, self.path)
        self.logger.info("Adding Services to the Ble Application")


        for service in p_services:
            self.add_service(service)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(bluetooth_constants.DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        self.logger.info('GetManagedObjects')

        for service in self.services:
            self.logger.info("GetManagedObjects: service="+service.get_path())
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()

        return response

class BleServerController:
    def __init__(self, register_client_callback = None):
        self.logger = logy.get_or_create_logger("BleServer", logy.DEBUG, "BLE")
        self._ids = 0
        self._connections = 1
        self._callbacks : Dict[int, Callable] = {}
        self._register_client_callback = register_client_callback

        ##### dbus Stuff ####
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        self._bus = dbus.SystemBus()
        # we're assuming the adapter supports advertising
        self._adapter_path = bluetooth_constants.BLUEZ_NAMESPACE + bluetooth_constants.ADAPTER_NAME
        self._adv_mgr_interface = dbus.Interface(self._bus.get_object(bluetooth_constants.BLUEZ_SERVICE_NAME, self._adapter_path), bluetooth_constants.ADVERTISING_MANAGER_INTERFACE)

        self._service_manager = dbus.Interface(
                self._bus.get_object(bluetooth_constants.BLUEZ_SERVICE_NAME, self._adapter_path),
                bluetooth_constants.GATT_MANAGER_INTERFACE)

        self._bus.add_signal_receiver(self.properties_changed,
                dbus_interface = bluetooth_constants.DBUS_PROPERTIES,
                signal_name = "PropertiesChanged",
                path_keyword = "path")

        self._bus.add_signal_receiver(self.interfaces_added,
                dbus_interface = bluetooth_constants.DBUS_OM_IFACE,
                signal_name = "InterfacesAdded")


        self._adv_mgr_interface = dbus.Interface(self._bus.get_object(bluetooth_constants.BLUEZ_SERVICE_NAME, self._adapter_path), bluetooth_constants.ADVERTISING_MANAGER_INTERFACE)
        self._adv = Advertisement(self._bus, 0, 'peripheral')
        self._adv.set_service_uuid(UUIDS[0])
        self.start_advertising()

        self._services = {}
        for i, uuid in enumerate(UUIDS):
            socket = BleServerSocket()
            ble_service = BleServerService(self._bus, '/org/bluez/ldsg', i, uuid)
            socket._factory = self
            socket.sendMessage = ble_service._characteristic.send_msg
            ble_service._characteristic._on_msg = socket.onMessage
            ble_service._characteristic._on_connect = socket.onConnect
            self._services[uuid] = (ble_service, socket)
        services = [serice_pair[0] for serice_pair in self._services.values()]
        self._app = Application(self._bus, services)

        self.logger.info('Registering GATT application...')


    def register_callback(self, message_code : int, callback : Callable):
        self._callbacks[message_code] = callback

    def get_id(self) -> str:
        self._ids += 1
        return f"user_{self._ids}"

    def run(self):
        self.logger.info("Start Loop")
        self._mainloop = GLib.MainLoop()
        self._service_manager.RegisterApplication(self._app.get_path(), {},
                                        reply_handler=self.register_app_cb,
                                        error_handler=self.register_app_error_cb)
        self._mainloop.run()

    def register_ad_cb(self):
        self.logger.info('Advertisement registered OK')

    def register_ad_error_cb(self, error):
        self.logger.info('Error: Failed to register advertisement: ' + str(error))
        self._mainloop.quit()

    def register_app_cb(self):
        self.logger.info('GATT application registered')

    def register_app_error_cb(self, error):
        self.logger.info('Failed to register application: ' + str(error))
        self._mainloop.quit()

    def set_connected_status(self, status):
        if (status == 1):
            self.logger.info("connected")
            self.stop_advertising()
            self._adv.set_service_uuid(UUIDS[self._connections])
            self._connections += 1
            self.start_advertising()
        else:
            self.logger.info("disconnected")

    def properties_changed(self, interface, changed, invalidated, path):
        if (interface == bluetooth_constants.DEVICE_INTERFACE):
            if ("Connected" in changed):
                self.set_connected_status(changed["Connected"])

    def interfaces_added(self, path, interfaces):
        if bluetooth_constants.DEVICE_INTERFACE in interfaces:
            properties = interfaces[bluetooth_constants.DEVICE_INTERFACE]
            if ("Connected" in properties):
                self.set_connected_status(properties["Connected"])

    def stop_advertising(self):
        self.logger.info("Unregistering advertisement", self._adv.get_path())
        self._adv_mgr_interface.UnregisterAdvertisement(self._adv.get_path())

    def start_advertising(self):
        self.logger.info("Registering advertisement", self._adv.get_path())
        self._adv_mgr_interface.RegisterAdvertisement(self._adv.get_path(), {},
                                            reply_handler=self.register_ad_cb,
                                            error_handler=self.register_ad_error_cb)

