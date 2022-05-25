import time
import roboticstoolbox as rtb
import numpy as np

from PyQt5.QtCore import pyqtSignal, QObject
from modifyed_swift import Swift
from spatialmath import SE3
from queue import Empty
from communication import server_tcp



class Simulation(QObject):

    signal_data = pyqtSignal()

    _finnish_ = False
    _RUN_ = True
    home_p = [0.30000024, 0.2900002, 0.13999661, -2.11982589, 1.53000467, 1.51018242] # home cartesianas
    home_q = [0.518,-1.476,2.278,-2.394,-1.538,-3.984] # home articulares

    def __init__(self):
        super().__init__()        

        self.backend = Swift()
        self.backend.launch(new_tab=False)
        self.backend.realtime = True


    def _init_(self):

        try:
            print("[#]: Esperando a detectar la aplicacion!")
            self.backend.inq.get()
            print("[#]: Ventana detectada!")
        except Empty:
            print('\n[#]: No se ha podido conectar con el simulador! \n')
            raise
        
        # Creamos el robot.
        self.robot = rtb.roboticstoolbox.models.UR5()
        self.robot.q = self.home_q
        self.desired_q = self.robot.q # Inicializamos la posición deseada con la actual del robot.
        self.robot.base = SE3(0,0,0)

        # Añadimos al robot
        self.backend.add(self.robot)

        self.run()


    def run(self):
        self.com = server_tcp(port=12345, controller_ip="localhost")
        
        # A la espera de poder conectarse con 2 clientes:
        while self._RUN_ and self.com.connected < 1:
            self.com.listen_for_connections(blocking=False)
        
        
        self.desired_q = self.robot.q   # Inicializamos la posicion deseada. 
        self.get_pos()                  # Obtenemos la posición actual del robot simulado.
        self.signal_data.emit()         # Emitimos señal para actualizar el panel de info.


        print("[#]: Running...")
        while self._RUN_:
            
            # Recibimos datos de control.
            data = self.com.recibir(self.com.detector)

            if data.any(): 

                # Posición deseada + offset.
                data = np.array(self.home_p) + data
                self.desired_q = self.move_to(data)

                # Actualizamos posición del robot y obtenemos la cinemática.
                self.robot.q = self.desired_q
                self.get_pos()

                self.signal_data.emit() # Emitimos señal para actualizar el panel de info.


            self.backend.step() # Send new frame.


        print("[#]: FINISHED")
        self._finnish_ = True
        

    def move_to(self, D_q):
        x, y, z, _, _, _ = D_q
        Rx,Ry,Rz = self.home_p[3:] # Dejamos fija la orientación del extremo.
        
        # CÁLCULO CINEMÁTICA INVERSA
        T = SE3(x,y,z)*SE3.RPY((Rx,Ry,Rz)) # Obtenemos la matriz de transformacion:
        
        start = time.time()
        sol = self.robot.ikine_min(T, q0=np.array(self.robot.q), ilimit=200)
        elapsed_time = time.time() - start
        print("[#]: Tiempo para calcular la cinematica inversa:", "{:.3}".format(elapsed_time), "s")


        # EVALUAMOS SOLUCIÓN
        if(not sol.success):
            print("[#] ERROR: El error de posicion es:", sol.residual)
            sol = [x,y,z,Rx,Ry,Rz]
        else:
            sol = sol.q
            
        return sol


    def get_pos(self):
        #Obtener datos posición:
        T = self.robot.fkine(self.robot.q)
        pos = T.t
        rot = T.rpy()
        art = np.array(self.robot.q)

        self.robot_pos = np.append(art,pos)
        self.robot_pos = np.append(self.robot_pos,rot)


    def stop(self):
        self._RUN_ = False