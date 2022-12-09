"""Simple UDP/IP client"""
import socket
import time

"""Client"""
# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Connect the socket to the port where the server is listening
server_address = ("127.0.0.1", 8889)

while True:
    sock.sendto(bytes("command", encoding='utf-8'), server_address)
    data, server = sock.recvfrom(1024)
    if not data:
        break
    print(data)
    time.sleep(0.1)

print('closing socket')
sock.close()
