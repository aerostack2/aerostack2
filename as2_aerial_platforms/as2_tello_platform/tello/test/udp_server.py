# echo-server.py

import socket
import time

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 8889  # Port to listen on (non-privileged ports are > 1023)

state_msg = "pitch:0;roll:0;yaw:0;vgx:0;vgy:0;vgz:0;templ:60;temph:63;tof:10;h:0;bat:2;baro:569.18;time:0;agx:-5.00;agy:-2.00;agz:-998.00;\r\n"

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    print(f"Biding to {HOST}:{PORT}")
    s.bind((HOST, PORT))

    while True:
        message, address = s.recvfrom(1024)
        if message:
            print(message)
            s.sendto(bytes("ok", encoding='utf-8'), address)
        time.sleep(0.1)

        s.sendto(bytes(state_msg, encoding='utf-8'), (HOST, 8890))
