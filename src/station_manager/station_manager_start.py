#!/usr/bin/python3
import sys
import signal
import argparse
import rospy
import logy
import pathlib
from src.station_manager import StationManager
from twisted.internet import reactor
from src import DataManager
from src.server import WebServerController, WebServerSocket, BleServerController
from typing import List

USE_BLE = True


def signal_handler(signal, frame):
    for socket in sockets:
        socket.kill()

def main():
    rospy.init_node('station_manager', anonymous=False)
    signal.signal(signal.SIGINT, signal_handler)
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", help="Verbose mode.", action="store_true")
    parser.add_argument("-s", "--showroom-mode", help="Verbose mode.", action="store_true")
    parser.add_argument("-d", "--debug-frames", default=0, type=int, help="Debug Frame time in ms. At 0 there are no debug frames.")
    parser.add_argument("--without-gui", action="store_true", help="Start without gui")

    arg_count = len(sys.argv)
    last_arg = sys.argv[arg_count - 1]

    if last_arg[:2] == "__":
        valid_args = sys.argv[1:arg_count - 2]
        args = parser.parse_args(valid_args)
    else:
        args = parser.parse_args()

    logy.Logy().basic_config(debug_level=logy.DEBUG, module_name="SM")
    logy.basic_config(debug_level=logy.DEBUG, module_name="SM")

    camera_path = str(pathlib.Path(__file__).absolute().parent.parent) + "/infrastructure/CameraNode.py"
    transform_node_path = str(pathlib.Path(__file__).absolute().parent) + "/launch/static_transform.launch"
    station_selection_path = str(pathlib.Path(__file__).absolute().parent) + "/station_selection.py"

    with_gui = not args.without_gui
    data_manager = DataManager()

    if USE_BLE:
        sockets.append(BleServerController())
    else:
        controller = WebServerController("ws://127.0.0.1:3030")
        controller.protocol = WebServerSocket
        sockets.append(controller)

    station_manager = StationManager(camera_path, transform_node_path, station_selection_path, data_manager=data_manager,
        verbose=True, debug_frames_ms=args.debug_frames, with_gui=with_gui, showroom_mode=args.showroom_mode, server_sockets=sockets)
    logy.info("Station Manager is Ready")

    if USE_BLE:
        sockets[0].run()
    else:
        reactor.listenTCP(3030, sockets[0])
        reactor.run()

    ## Use Station Manager Directly ##
    # def repetition_callback(response_code=508, satus_code=2, payload=dict({})):
    #     print("repetitions =", payload["repetitions"])
    #     print("exercise =", payload["exercise"])
    #     print("set id =", payload["set_id"])

    # my_id = "id124"
    # data_manager = DataManager()
    # station_manager = StationManager(debug_mode=args.debug, verbose=args.verbose, data_manager=data_manager)
    # station_manager.set_client_callback(my_id, repetition_callback)
    # station_manager.login_station(my_id, 2)
    # station_manager.start_exercise(my_id, 2, 105)
    # time.sleep(5)
    # station_manager.stop_exercise(my_id)
    # station_manager.logout_station(my_id)
if __name__ == '__main__':
    sockets = []
    main()
