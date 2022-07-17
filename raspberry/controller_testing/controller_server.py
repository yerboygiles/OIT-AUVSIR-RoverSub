import socket

ip = '127.0.0.1' # this IP of my pc. When I want raspberry pi 4`s as a server use "192.168.137.31"
port = 1234
BUFFER_SIZE = 8192 # Normally 1024, but I want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', port))
s.listen(1)

conn, addr = s.accept()
print ('Connection address:', addr)
while 1:
    data = conn.recv(BUFFER_SIZE)
    if not data: break
    print ("received data:", data)
    conn.send(data)  # echo
conn.close()