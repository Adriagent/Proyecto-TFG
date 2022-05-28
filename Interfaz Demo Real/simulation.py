import time
import roboticstoolbox as rtb
import numpy as np

from numpy import pi
from PyQt5.QtCore import pyqtSignal, QObject
from spatialmath import SE3
from queue import Empty
from communication import server_tcp

from roboticstoolbox.backends.PyPlot import PyPlot
from modifyed_swift import launch, add, step



class Simulation(QObject):

    signal_data = pyqtSignal()

    _finnish_ = False
    _RUN_ = True
    home_p = [-0.11841544, -0.26807058, 0.15727483, 3.11652424, 0.02504716, -3.14082373]    # home cartesianas.
    home_q = np.deg2rad([-91.71, -98.96, -126.22, -46.29, 91.39, 358.21])                   # home articulares.
    lol = home_q.copy()

    def __init__(self, figure, canvas):
        super().__init__()  

        self.canvas = canvas
        self.figure = figure
        self.env = PyPlot()

        # Creamos el robot.
        self.robot = rtb.DHRobot(
            [
                rtb.RevoluteDH(d=0.1519, alpha=pi/2, qlim=np.deg2rad([-363, 363])), # Base
                rtb.RevoluteDH(a=-0.24365, qlim=np.deg2rad([-363, 363])), # Hombro (realmente de -364 a 363)
                rtb.RevoluteDH(a=-0.21325, qlim=np.deg2rad([-363, 363])), # Codo
                rtb.RevoluteDH(d=0.11235, alpha=pi/2, qlim=np.deg2rad([-363, 363])), # Muñeca 1
                rtb.RevoluteDH(d=0.08535, alpha=-pi/2, qlim=np.deg2rad([-363, 363])), # Muñeca 2
                rtb.RevoluteDH(d=0.0819, qlim=np.deg2rad([-1000, 1000])) # Muñeca 3 (sin límites articuares)
            ], name="UR3")

        self.robot.q = self.home_q
        self.desired_q = self.robot.q # Inicializamos la posición deseada con la actual del robot.

        launch(self.env, fig=self.figure, limits=[-0.5,0.5,-0.5,0.5,-0.5,0.5])
        add(self.env, self.robot, readonly=True)
        step(self.env)



    def _init_(self):

        try:
            print("[#]: Esperando a detectar la aplicacion!")
            print("[#]: Ventana detectada!")
        except Empty:
            print('\n[#]: No se ha podido conectar con el simulador! \n')
            raise
        
        # Añadimos al robot
        self.run()


    def run(self):
        self.desired_q = self.robot.q   # Inicializamos la posicion deseada. 
        self.get_pos()                  # Obtenemos la posición actual del robot simulado.
        self.signal_data.emit()         # Emitimos señal para actualizar el panel de info.


        self.com = server_tcp(port=50002, controller_ip='')
        
        # A la espera de poder conectarse con 2 clientes:
        while self._RUN_ and self.com.connected < 2:
            self.com.listen_for_connections(blocking=False)
        

        print("Moviendo robot a posición inicial:")
        msg = str(self.home_q)
        self.com.demo.send(msg.encode())
        msg = self.com.demo.recv(1024).decode() # recibimos confirmación del robot real.

        self.com.enviar(self.com.detector, "empezar")
 

        print("[#]: Running...")
        while self._RUN_:
            
            # Recibimos datos de control.
            data = self.com.recibir(self.com.detector)

            if data.any(): 

                # Posición deseada + offset.
                data = np.array(self.home_p) + data
                self.desired_q = self.move_to(data)

                # Actualizamos posición del robot y obtenemos la cinemática.
                self.com.enviar(self.com.demo, self.desired_q)

                msg = self.com.demo.recv(1024).decode()
                temp = np.array(data).astype(float)

                self.robot.q = self.lol
                self.get_pos()

                self.signal_data.emit() # Emitimos señal para actualizar el panel de info.

            # self.lol[1]+=0.001
            # step(self.env, 0.01)                


        print("[#]: FINISHED")
        self._finnish_ = True
        

    def move_to(self, D_q):
        x, y, z, _, _, _ = D_q
        Rx,Ry,Rz = self.home_p[3:] # Dejamos fija la orientación del extremo.
        
        # CÁLCULO CINEMÁTICA INVERSA
        T = SE3(x,y,z)*SE3.RPY((Rx,Ry,Rz)) # Obtenemos la matriz de transformacion:
        
        start = time.time()
        sol = self.robot.ikine_min(T, q0=self.home_q, ilimit=200, end=self.robot.ee_links[0])
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
        T = self.robot.fkine(self.robot.q, end=self.robot.ee_links[0])
        pos = T.t
        rot = T.rpy()
        art = np.array(self.robot.q)

        self.robot_pos = np.append(art,pos)
        self.robot_pos = np.append(self.robot_pos,rot)


    def stop(self):
        self._RUN_ = False
