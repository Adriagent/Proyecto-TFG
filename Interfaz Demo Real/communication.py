import socket
import numpy as np

class server_tcp:
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    connected = 0

    def __init__(self, port, controller_ip):
        server_address = ('', port)

        self.CONTROLLER_IP = controller_ip

        self.sock.bind(server_address)

        print('[#]: Socket a la espera de clientes...')
        self.sock.listen(2)


    def listen_for_connections(self, blocking = True):

        self.sock.setblocking(blocking)
        
        for _ in range(2):
            try:
                connection, address = self.sock.accept()

                print('\t- Conectado a: ' + str(address[0]), end=" ")
                self.connected+=1

                if self.connected == 1:
                    self.detector = connection
                    print("(Detector de pose)")
                elif self.connected == 2:
                    self.demo = connection
                    print("(Robot real)")

            except BlockingIOError:
                self.sock.setblocking(True)
                

        self.sock.setblocking(True)


    def enviar(self, client, msg):

        try:
            client.send(str(msg).encode())
        except ConnectionResetError:
            print("[#]: Conexion cerrada!")


    def recibir(self,client):

        data = np.array([])

        try:
            size = int(client.recv(3).decode())

            data = client.recv(size).decode().split()
            data = np.array(data).astype(float) # Vector de datos. (x,y,z, rx,ry,rz)
       
        except(ConnectionResetError, ValueError):
            print("[#]: Conexion cerrada!")

        return data
    

    def __del__(self):
        print("[#]: Cerrando Servidor...")
        self.sock.close()


class client_tcp:

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    __is_open = False

    def __init__(self,server_host, port):
        server_address = (server_host, port)

        print('[#]: Conectandose a {} port {}'.format(*server_address))
        self.sock.connect(server_address) # A lo mejor renta hacerlo un bucle para que se quede esperando.
        print("[#]: Conectado al servidor!")
        self.__is_open = True


    def enviar(self, q):
        message = " ".join(str(round(x,5)) for x in q)
        MESSAGE = str(len(message)) + " " + message
        MESSAGE = MESSAGE.encode()
        try:
            self.sock.sendall(MESSAGE)
        except ConnectionResetError:
            print("[#]: Conexion cerrada!")
            self.__is_open = False


    def recibir(self):

        data = [0,0,0,0,0,0]

        try:
            size = int(self.sock.recv(3).decode())

            data = self.sock.recv(size).decode().split()
            data = np.array(data).astype(float) # Vector de datos. (x,y,z, rx,ry,rz)
       
        except (ConnectionResetError, ValueError):
            print("[#]: Conexion cerrada!")
            self.__is_open = False


        return data


    def is_open(self):
        return self.__is_open


    def __del__(self):
        print("[#]: Cerrando Cliente...")
        self.sock.close()



