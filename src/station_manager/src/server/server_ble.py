#!/usr/bin/python3
# Advertises, accepts a connection, creates the microbit temperature service, LED service and their characteristics
from asyncio.log import logger
import os
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
import subprocess
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
from .abstract_server import ServerController, ServerSocket
import time
from gymy_tools import TwoWayDict
sys.path.insert(0, '.')

UUIDS = ["ac00", "ac01", "ac02", "ac03"]
# much of this code was copied or inspired by test\example-advertisement in the BlueZ source

class BleServerSocket(ServerSocket):
    def __init__(self):
        pass

def get_long_uuid(uuid):
    if len(uuid) == 4:
        uuid_short = uuid.zfill(8)
        complete_uuid = f"{uuid_short}-0000-1000-8000-00805f9b34fb"
        return complete_uuid
    return uuid

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
            complete_uuid = get_long_uuid(uuid)
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
        bluetooth_gatt.Service.__init__(self, bus, path_base, index, get_long_uuid(uuid), True)
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
        self._on_msg = None
        self._on_connect = None
        self.logger = logy.get_or_create_logger("BleServer", logy.DEBUG, "BLE")
        self.logger.info(f"Adding Characteristic uuid {uuid}")

    def WriteValue(self, value, options):
        self.logger.warn(str(options))
        ascii_bytes = bluetooth_utils.dbus_to_python(value)
        text = bytes(ascii_bytes)
        self.logger.info(f"Asciii bytes: {text}")
        self.logger.info("Write")
        if self._on_msg is not None:
            self.logger.info("Write callback")
            self._on_msg(text, False)

    def ReadValue(self, options):
        self.logger.warn(str(options))
        self.logger.info('ReadValue in BleServer called')
        value = []
        value.append(dbus.int32(10))
        return value

    def send_msg(self, text, is_byte) -> bool:
        self.logger.info(f"send_msg in characteristics: {text}")
        if is_byte:
            self.logger.error(f"is_byte is not supported")
            return False

        self.logger.info(f"try notifying {text} / {type(text)}")
        values = text #.encode("utf-8")
        value = []
        for byte in values:
            value.append(dbus.Byte(byte))

        self.logger.info(f"notifying")
        self.PropertiesChanged(bluetooth_constants.GATT_CHARACTERISTIC_INTERFACE, { 'Value': value }, [])
        return True

    def StartNotify(self):
        self.logger.info("starting notifications")
        if self._on_msg is not None:
            self._on_connect()

    def StopNotify(self):
        self.logger.info("stopping notifications")


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

class BleServerController(ServerController):
    def __init__(self):
        logger = logy.get_or_create_logger("BleServer", logy.DEBUG, "BLE")
        ServerController.__init__(self, logger)
        self._connections : Dict[str, str] = TwoWayDict()
        self._advertised_uuid : str = ""
        #list_files = subprocess.run(["echo",  "1",  ">", "/home/trainerai/trainerai-core/bt_restart"])
        #os.system("echo 1 > /home/trainerai/trainerai-core/bt_restart")
        time.sleep(2)
        logger.debug("Bluetooth restarted")

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
        self._advertised_uuid = UUIDS[0]
        self.start_advertising()

        self._services = {}
        for i, uuid in enumerate(UUIDS):
            socket = BleServerSocket()
            ble_service = BleServerService(self._bus, '/org/bluez/ldsg', i, uuid)
            socket.init_socket(self, ble_service._characteristic.send_msg, self._logger)
            ble_service._characteristic._on_msg = socket._on_message
            ble_service._characteristic._on_connect = socket._on_connection
            self._services[uuid] = (ble_service, socket)
        services = [serice_pair[0] for serice_pair in self._services.values()]
        self._app = Application(self._bus, services)

        self._logger.info('Registering GATT application...')

    def run(self):
        self._logger.info("Start Loop")
        self._mainloop = GLib.MainLoop()
        self._service_manager.RegisterApplication(self._app.get_path(), {},
                                        reply_handler=self.register_app_cb,
                                        error_handler=self.register_app_error_cb)
        self._mainloop.run()

    def kill(self):
        self._mainloop.quit()

    def register_ad_cb(self):
        self._logger.info('Advertisement registered OK')

    def register_ad_error_cb(self, error):
        self._logger.info('Error: Failed to register advertisement: ' + str(error))
        self._mainloop.quit()

    def register_app_cb(self):
        self._logger.info('GATT application registered')

    def register_app_error_cb(self, error):
        self._logger.info('Failed to register application: ' + str(error))
        self._mainloop.quit()

    def set_connected_status(self, status : int, path : str):
        if (status == 1):
            self._logger.info("connected " + str(path))
            self.stop_advertising()
            if self._advertised_uuid in self._connections:
                self._logger.error(f"More than 1 Connection on UUID {self._advertised_uuid}")
            if path in self._connections:
                self._logger.error(f"User {path} is loged in multiple times")
            self._logger.info(f"advertised uuid: {self._advertised_uuid}")
            self._connections[self._advertised_uuid] = path
            self._advertised_uuid = ""
            for uuid in UUIDS:
                if uuid not in self._connections:
                    self._logger.info(f"advertise new uuid: {uuid}")
                    self._advertised_uuid = uuid
                    self._adv.set_service_uuid(uuid)
                    self.start_advertising()
                    break
        else:
            self._logger.info("disconnected")
            if path not in self._connections:
                self._logger.error(f"User not logged in properly")
                return
            if not self._advertised_uuid:
                self._advertised_uuid = self._connections[path]
                self._adv.set_service_uuid(self._advertised_uuid)
                self.start_advertising()
            del self._connections[path]

    def properties_changed(self, interface, changed, invalidated, path):
        if (interface == bluetooth_constants.DEVICE_INTERFACE):
            if ("Connected" in changed):
                self.set_connected_status(changed["Connected"], path)

    def interfaces_added(self, path, interfaces):
        if bluetooth_constants.DEVICE_INTERFACE in interfaces:
            properties = interfaces[bluetooth_constants.DEVICE_INTERFACE]
            if ("Connected" in properties):
                self.set_connected_status(properties["Connected"], path)

    def stop_advertising(self):
        self._logger.info("Unregistering advertisement", self._adv.get_path())
        self._adv_mgr_interface.UnregisterAdvertisement(self._adv.get_path())

    def start_advertising(self):
        self._logger.info("Registering advertisement", self._adv.get_path())
        self._adv_mgr_interface.RegisterAdvertisement(self._adv.get_path(), {},
                                            reply_handler=self.register_ad_cb,
                                            error_handler=self.register_ad_error_cb)

