import socket

HOST = "143.27.18.83"
#HOST = "143.27.190.130" (my laptop)
PORT = 1978

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('socket created')
sock.connect((HOST,PORT))
print('socket connected')
print(sock)
#msg  = b"Ping<EOF>"
msg  = b"StartStrength<EOF>"
sock.send(msg)
print('socket msg sent: {}'.format(msg))
data = sock.recv(1024)
print('socket msg received: {}'.format(data))
sock.close()

print(data)
print(data.decode)