from twisted.internet import reactor
from .server import ServerController, ServerSocket, ResponseAnswer

def login_station(user_id, payload):
    print("login into station. Payload: ", payload)
    return ResponseAnswer(501, 1)

def logout_station(user_id, payload):
    print("logout from station")
    return ResponseAnswer(501, 1)

def start_exercise(user_id, payload):
    print("start exercise")
    return ResponseAnswer(501, 1)

def stop_exercise(user_id, payload):
    print("stop exercise")
    return ResponseAnswer(501, 1)

if __name__ == '__main__':
    server_controller = ServerController("ws://127.0.0.1:9000")
    server_controller.register_callback(1, login_station)
    server_controller.register_callback(2, logout_station)
    server_controller.register_callback(3, start_exercise)
    server_controller.register_callback(4, stop_exercise)
    server_controller.protocol = ServerSocket
    reactor.listenTCP(9000, server_controller)
    reactor.run()