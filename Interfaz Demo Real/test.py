import socket
import numpy as np
from time import time

def stop(c,s):
    c.close()
    s.close()
    print("[#] Servidor finalizado.")
    exit()


home_q = np.deg2rad([-91.71, -98.96, -126.22, -46.29, 91.39, 358.21])
# home_q = np.deg2rad([29.67921379, -84.56857056, 130.51978573, -137.16609615, -88.12090889, -228.26638558])

HOST = '' #'192.168.2.130'
PORT = 50002
print("[#] Iniciando servidor")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

try:
    s.bind((HOST,PORT))
    print("[#] Escuchando en", HOST, ":", PORT)
    s.listen(5)
    c, addr = s.accept()
    print("[#] Cliente conectado!", addr)
except KeyboardInterrupt:
    exit("")

## setup.
try:

    msg = str(home_q)
    print("[#] Moviendo a home:")
    c.send(msg.encode()) # enviamos posición inicial home.

    msg = c.recv(1024).decode() # recibimos confirmación.
    print(msg)

except BrokenPipeError:     
    print("[!] Cliente desconectado")
    stop(c,s)
except KeyboardInterrupt:   
    stop(c,s)


try:
    last = time()
    while True:
        if time() - last >= 0.05:
            msg = str(home_q)
            c.send(msg.encode())
            msg = c.recv(1024).decode() # recibimos posicion actual.
            print(msg)

            last = time()
            home_q[1]+=0.005
            home_q[2]+=0.005

except BrokenPipeError:     
    print("[!] Cliente desconectado")
    stop(c,s)
except KeyboardInterrupt:   
    stop(c,s)



