#!/usr/bin/env python3
"""PyBluez simple example rfcomm-server.py
Simple demonstration of a server application that uses RFCOMM sockets.
Author: Albert Huang <albert@csail.mit.edu>
$Id: rfcomm-server.py 518 2007-08-10 07:20:07Z albert $
"""
from urllib import response
import bluetooth
import json
import time

server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_sock.bind(("", bluetooth.PORT_ANY))
server_sock.listen(8)

port = server_sock.getsockname()[1]

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

bluetooth.advertise_service(server_sock, "SampleServer", service_id=uuid,
                            service_classes=[uuid, bluetooth.SERIAL_PORT_CLASS],
                            profiles=[bluetooth.SERIAL_PORT_PROFILE],
                            # protocols=[bluetooth.OBEX_UUID]
                            )

print("Waiting for connection on RFCOMM channel", port)

client_sock, client_info = server_sock.accept()
client_sock.settimeout(0)
print("Accepted connection from", client_info)


while True:
    try:
        data = client_sock.recv(1024)

    except:
        # print("no data")
        # print("sleep 5")
        try:
            client_sock.getpeername()
            print("connected")
        except:
            print("disconnected")
        time.sleep(1)
        continue
    print(data)
    # resnponse = {}
    # resnponse["id"] = "user123"
    # resnponse["type"] = 0
    # resnponse["payload"] = {"station" : 1, "exercise" : 105}
    # resnponse = json.dumps(resnponse, ensure_ascii=True).encode('ascii')
    resnponse = "Check"
    client_sock.send(resnponse)
    if not data:
        break
    print("Received", data)
    print("Sent", resnponse)



print("Disconnected.")

client_sock.close()
server_sock.close()
print("All done.")