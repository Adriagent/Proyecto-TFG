import sys
import numpy as np

import warnings
import rospy

from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QApplication, QVBoxLayout, QGridLayout, QTextEdit, QWidget, QLabel
from PyQt5.QtCore import QThread, QUrl, Qt, pyqtSignal
from PyQt5.QtWebEngineWidgets import QWebEngineView
from simulation import Simulation

warnings.filterwarnings("ignore", message="delta_grad == 0.0. Check if the approximated function is linear.")


class Aplicacion(QWidget):

    app = QApplication(sys.argv)
    stop_signal = pyqtSignal()

    def __init__(self):
        super().__init__()

        self.sim = Simulation() # iniciamos la simulacion.

        web = QWebEngineView()
        web.load(QUrl('http://localhost:'+ str(self.sim.backend.server_port)+ '/?'+ str(self.sim.backend.socket_port)))
        web.setAttribute(Qt.WA_StyledBackground)
        web.setStyleSheet("border: 2px solid black;")
        web.setContentsMargins(1, 1, 1, 1)

        web.show()

        self.setWindowTitle("Interfaz")
        
        layout = QGridLayout()

        grid_position = QGridLayout() 
        grid_desired = QVBoxLayout()
        
        vert0 = QVBoxLayout()   # Columna de texto.
        vert1 = QVBoxLayout()   # Columna posiciones remoto.



        self.alert_text = QLabel("INVALID CONFIGURATION!")
        self.alert_text.setStyleSheet("font-weight: bold; color: #FF0000")
        self.alert_text.setAlignment(Qt.AlignCenter)
        self.alert_text.setVisible(False)


        labels = ["J0", "J1", "J2", "J3", "J4", "J5", "X", "Y", "Z", "ROLL", "PITCH", "YAW"]

        self.tags = []
        self.art1 = []
        self.art2 = []

        for i in range(13):

            if i == 0:
                temp0 = QLabel("ARTICULAR")
                temp0.setAlignment(Qt.AlignCenter)
                temp0.setStyleSheet("font-weight: bold; color: #FFFFFF")
                vert0.addWidget(temp0)

                temp1 = QLabel(" ")
                temp1.setAlignment(Qt.AlignCenter)
                temp1.setStyleSheet("font-weight: bold; color: #FFFFFF")
                vert1.addWidget(temp1)

            else:
                n = i-1
                self.tags.append(QTextEdit(labels[n]))
                self.tags[n].setFixedWidth(100)
                self.tags[n].setReadOnly(True)
                self.tags[n].setFrameStyle(1)
                self.tags[n].setAlignment(Qt.AlignCenter)
                self.tags[n].setStyleSheet("background-color: #3D3D3D; font-weight: bold; color: #FFFFFF")
                vert0.addWidget(self.tags[n])

                self.art1.append(QTextEdit("-"))
                self.art1[n].setFixedWidth(100)
                self.art1[n].setReadOnly(True)
                self.art1[n].setAlignment(Qt.AlignCenter)
                self.art1[n].setStyleSheet("background-color: #3D3D3D; font-weight: bold; color: #FFFFFF")
                vert1.addWidget(self.art1[n])

            if i == 6 or i == 9:
                space = QLabel("CARTESIAN")
                space.setAlignment(Qt.AlignCenter)
                space.setStyleSheet("font-weight: bold; color: #FFFFFF")

                if i == 9:
                    space.setText("ORIENTATION")
                
                space.setFixedSize(100,40)
                vert0.addWidget(space)
                vert1.addWidget(space)


        grid_position.addLayout(vert0, 0,0)
        grid_position.addLayout(vert1, 0,1)


        layout.addLayout(grid_position, 0,0)
        layout.addWidget(web,  0,1)
        layout.addLayout(grid_desired,1,1)

        self.setLayout(layout)
        self.setFixedSize(1000,600)  # TAMAÑO DE LA VENTANA FIJO.

        p = self.palette()
        p.setColor(self.backgroundRole(), QColor(51,51,51)) # Definimos el color de fondo de la aplicación.
        self.setPalette(p)

        self.show()
        self.init_threads()


    def run(self):

        self.thread.start() # Lanzamos el sim._init_ en otro hilo.
        self.app.exec()     # Lanzamos la aplicación.


    def init_threads(self):

        self.thread = QThread()

        self.sim.moveToThread(self.thread)              # Movemos la ejecución del objeto a otro hilo.
        self.stop_signal.connect(self.sim.stop)
        self.thread.started.connect(self.sim._init_)    # La señal started del thread lanzará la función _init_ de la simulación.

        # Emision de datos y otras cosas.
        self.sim.signal_data.connect(self.update_text)      # Cada vez que enviemos datos los recibe esta funcion.


    def update_text(self):

        # real
        for i, remote_art in enumerate(self.art1):
            if 5 < i < 9:
                remote_art.setText(str(round(self.sim.robot_pos[i],2)))
            else:
                temp = np.rad2deg(self.sim.robot_pos[i])
                temp = np.sign(temp)*((np.sign(temp) * temp)%360)
                remote_art.setText(str(round(temp,2)))

            remote_art.setAlignment(Qt.AlignCenter)     # TODO ESTO ES UNA PUTA GUARRADA.


    def closeEvent(self, _):
        self.sim.stop()
        while not self.sim._finnish_:
            None # Esperamos a que haya finalizado la ejecución del hilo.

        self.thread.quit()
        self.sim.deleteLater()
        self.thread.deleteLater()
        





if __name__ == "__main__":
    rospy.init_node("arm_controller")

    window = Aplicacion()

    window.run()




