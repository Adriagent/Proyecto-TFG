from concurrent.futures import thread
import numpy as np
import math
from numpy import pi
import roboticstoolbox as rtb
from spatialmath import SE3


home_q = np.deg2rad([-91.71, -98.97, -126.22, -46.29, 91.39, 358.21])


robot1 = rtb.DHRobot(
            [
                rtb.RevoluteDH(d=0.1519, alpha=pi/2, qlim=np.deg2rad([-363, 363])), # Base
                rtb.RevoluteDH(a=-0.24365, qlim=np.deg2rad([-363, 363])), # Hombro (realmente de -364 a 363)
                rtb.RevoluteDH(a=-0.21325, qlim=np.deg2rad([-363, 363])), # Codo
                rtb.RevoluteDH(d=0.11235, alpha=pi/2, qlim=np.deg2rad([-363, 363])), # Muñeca 1
                rtb.RevoluteDH(d=0.08535, alpha=-pi/2, qlim=np.deg2rad([-363, 363])), # Muñeca 2
                rtb.RevoluteDH(d=0.0819, qlim=np.deg2rad([-1000, 1000])) # Muñeca 3 (sin límites articuares)
            ], name="UR3")

robot2 = rtb.DHRobot(
            [
                rtb.RevoluteDH(d=0.15185, alpha=pi/2, qlim=np.deg2rad([-363, 363])), # Base
                rtb.RevoluteDH(a=-0.24355, qlim=np.deg2rad([-363, 363])), # Hombro (realmente de -364 a 363)
                rtb.RevoluteDH(a=-0.2132, qlim=np.deg2rad([-363, 363])), # Codo
                rtb.RevoluteDH(d=0.13105, alpha=pi/2, qlim=np.deg2rad([-363, 363])), # Muñeca 1
                rtb.RevoluteDH(d=0.08535, alpha=-pi/2, qlim=np.deg2rad([-363, 363])), # Muñeca 2
                rtb.RevoluteDH(d=0.0921, qlim=np.deg2rad([-1000, 1000])) # Muñeca 3 (sin límites articuares)
            ], name="UR3e")


robot3 = rtb.roboticstoolbox.models.UR3()

robot = robot1

print(robot)

T = robot.fkine(home_q)
R = T.R

theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
multi = 1 / (2 * math.sin(theta))

rx = multi * (R[2, 1] - R[1, 2]) * theta
ry = multi * (R[0, 2] - R[2, 0]) * theta
rz = multi * (R[1, 0] - R[0, 1]) * theta


np.set_printoptions(suppress=True)
print(np.round(T.t*1000,3), np.round([rx, ry, rz],3))

q = robot.ikine_min(T, q0=home_q, end=robot.ee_links[0]).q
print(np.rad2deg(q))



home_p = [-0.11841544, -0.26807058, 0.15727483, 3.11652424, 0.02504716, -3.14082373]

x, y, z, Rx, Ry, Rz = home_p

# CÁLCULO CINEMÁTICA INVERSA
T = SE3(x,y,z)*SE3.RPY((Rx,Ry,Rz)) # Obtenemos la matriz de transformacion:

sol = robot.ikine_min(T, q0=home_q, ilimit=200, end=robot.ee_links[0]).q
print(np.rad2deg(q))
# a = robot.plot(home_q)


temp = home_q

# importing various libraries
import sys
from PyQt5.QtWidgets import QDialog, QApplication, QPushButton, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
from roboticstoolbox.backends.PyPlot import PyPlot
from modifyed_swift import launch, add, step  



class Window(QWidget):
      
    # constructor
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)
  
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)

        self.env = PyPlot()
        
        self.robot = robot1
        self.robot.q = temp

        launch(self.env, fig=self.figure)
        add(self.env, self.robot, readonly=True)
        step(self.env, 0.05)  


        self.canvas.setStyleSheet("border: 2px solid black;")
        self.canvas.setContentsMargins(1, 1, 1, 1)
  
        self.button = QPushButton('Plot')
        self.button.clicked.connect(self.plot)
  

        layout = QVBoxLayout()
                  
        layout.addWidget(self.canvas)
        layout.addWidget(self.button)
          
        self.setLayout(layout)

    def plot(self):

        for i in range(20):
            temp[1]+=0.5
            self.robot.q = temp
            step(self.env, 0.05)


  
# driver code
if __name__ == '__main__':
      
    # creating apyqt5 application
    app = QApplication(sys.argv)
  
    # creating a window object
    main = Window()
      
    # showing the window
    main.show()
  
    # loop
    sys.exit(app.exec_())

 




