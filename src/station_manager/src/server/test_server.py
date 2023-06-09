from twisted.internet import reactor
from .server_websocket import ServerController, ServerSocket, SMResponse

def login_station(user_id, payload):
    print("login into station. Payload: ", payload)
    return SMResponse(501, 1)

def logout_station(user_id, payload):
    print("logout from station")
    return SMResponse(502, 1)

def start_exercise(user_id, payload):
    print("start exercise")
    return SMResponse(503, 1)

def stop_exercise(user_id, payload):
    print("stop exercise")
    return SMResponse(504, 1)

def weight_detection(user_id, payload):
    print("Start Weight Detection")
    return SMResponse(507, 1, {"weight" : 500.0, "probability" : 0.5})

if __name__ == '__main__':
    server_controller = ServerController("ws://127.0.0.1:3030")
    server_controller.register_callback(1, login_station)
    server_controller.register_callback(2, logout_station)
    server_controller.register_callback(3, start_exercise)
    server_controller.register_callback(4, stop_exercise)
    server_controller.register_callback(7, weight_detection)
    server_controller.protocol = ServerSocket
    reactor.listenTCP(3030, server_controller)
    reactor.run()