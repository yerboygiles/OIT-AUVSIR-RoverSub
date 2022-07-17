import socket

import byte as byte

ip = '127.0.0.1' # this IP of my pc. When I want raspberry pi 4`s as a client use "192.168.137.31"
port = 1234
BUFFER_SIZE = 8192
MESSAGE = "Hello, World!"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((ip, port))
s.send(MESSAGE)
data = s.recv(BUFFER_SIZE)
s.close()

print ("received data:", data)