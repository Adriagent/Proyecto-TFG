import time
import roboticstoolbox as rtb
import numpy as np

from queue import Empty
from spatialmath import SE3
import matplotlib.pyplot as plt
from PyQt5.QtCore import pyqtSignal, QObject
from roboticstoolbox.backends.PyPlot import PyPlot

from utils.communication import server_tcp
from utils.modifyed_swift import Swift
from utils.modifyed_pyplot import launch, add, step


class Simulation(QObject):

    signal_data = pyqtSignal()

    _finnish_ = False
    _RUN_ = True

    def __init__(self, robot=None, home_q=None):
        super().__init__()

        self.robot = robot

        if self.robot == None:
            self.robot = rtb.roboticstoolbox.models.UR5()


        if home_q is None:
            self.home_q = self.robot.q
        else:
            self.home_q = home_q


        T = self.robot.fkine(self.home_q)
        self.home_p = np.append(T.t,T.rpy())

        if isinstance(self.robot, rtb.DHRobot):
            self.backend = PyPlot()
        elif isinstance(self.robot, rtb.ERobot):
            if self.robot.hasgeometry:
                self.backend = Swift()
            else:
                self.backend = PyPlot()
        else:
            exit("[ERROR]: Modelo de robot no reconocido!")
            

        if isinstance(self.backend, Swift):
            self.backend.launch(new_tab=False) # Simulación en Swift.
            self.backend.realtime = True
        else:
            self.figure = plt.figure() # simulación en PyPlot.


    def _init_(self):
        self.robot.q = self.home_q
        self.desired_q = self.robot.q # Inicializamos la posición deseada con la actual del robot.
        

        if isinstance(self.backend, Swift):

            try:
                print("[#]: Esperando a detectar la aplicacion!")
                self.backend.inq.get()
                print("[#]: Ventana detectada!")
            except Empty:
                print('\n[#]: No se ha podido conectar con el simulador! \n')
                raise
        
            self.backend.add(self.robot)

        else:
            launch(self.backend, fig=self.figure, limits=[-0.5,0.5,-0.5,0.5,-0.5,0.5])
            add(self.backend, self.robot, readonly=True)
            step(self.backend)
            self.figure.canvas.start_event_loop(0.05)


        self.run()


    def run(self):
        self.desired_q = self.robot.q   # Inicializamos la posicion deseada. 
        self.get_pos()                  # Obtenemos la posición actual del robot simulado.
        self.signal_data.emit()         # Emitimos señal para actualizar el panel de info.



        self.com = server_tcp(port=50002)
        
        # A la espera de poder conectarse con 2 clientes:
        while self._RUN_ and self.com.connected < 2:
            self.com.listen_for_connections(blocking=False)
        
        if not self._RUN_:
            print("[#]: FINISHED")
            self._finnish_ = True
            return

        print("[#] Moviendo robot a posición inicial:")
        msg = str(self.home_q)
        self.com.demo.send(msg.encode())
        msg = self.com.demo.recv(1024).decode() # recibimos confirmación del robot real.

        print("\nempezando movimiento\n")

        self.com.enviar(self.com.detector, "empezar")


        print("[#]: Running...")
        while self._RUN_ and self.com.connected == 1:
            
            # Recibimos datos de control.
            data = self.com.recibir(self.com.detector)

            if data.any(): 

                # Posición deseada + offset.
                data = np.array(self.home_p) + data
                self.desired_q = self.move_to(data)

                # Actualizamos posición del robot real y obtenemos su posición real.
                self.com.enviar(self.com.demo, self.desired_q)
                msg = self.com.demo.recv(1024).decode()
                temp = np.array(data).astype(float)

                self.robot.q = temp

                self.get_pos()
                self.signal_data.emit() # Emitimos señal para actualizar el panel de info.


            if isinstance(self.backend, Swift):
                self.backend.step() # Send new frame.
            else:
                step(self.backend)
                self.figure.canvas.start_event_loop(0.05)

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