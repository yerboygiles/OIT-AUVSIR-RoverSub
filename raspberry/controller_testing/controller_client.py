import socket

TCP_IP = '64.112.174.137' # this IP of my pc. When I want raspberry pi 4`s as a server use "192.168.137.31"
TCP_PORT = 5005
BUFFER_SIZE = 1024 # Normally 1024, but I want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

conn, addr = s.accept()
print ('Connection address:', addr)
while 1:
    data = conn.recv(BUFFER_SIZE)
    if not data: break
    print ("received data:", data)
    conn.send(data)  # echo
conn.close()