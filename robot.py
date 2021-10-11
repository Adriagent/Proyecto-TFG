import rospy
from rospy.timer import sleep
from std_msgs.msg import String
import roboticstoolbox as rtb
from roboticstoolbox.backends.Swift import Swift
from spatialmath import SE3
import math
from math import pi


class RecibirMensajes:
    data = "none"

    def __init__(self):
        self.sub = rospy.Subscriber('/angulos_pose',String, self.callback)

    def callback(self,msg):
        self.data = msg.data



mensaje = RecibirMensajes()
anterior = mensaje.data

robot = rtb.roboticstoolbox.models.UR5()

rospy.init_node('simulacion_robot')

# Obtenemos la matriz de transformacion:
T = SE3(0,0,1)*SE3.RPY((0,0,0))
robot.base = T

#print(robot)

robot.q = [0, 0, 0, -pi/2, 0, 0]

#print(robot.q)

backend = Swift()
backend.launch()
backend.add(robot)

while not rospy.is_shutdown():

    if mensaje.data is not anterior:

        anterior = mensaje.data
        #print(mensaje.data)
        data = mensaje.data.split()
        data = list(map(float, data))
        phi = data[0]
        theta = data[1]
        print(math.degrees(phi), math.degrees(theta))
        robot.q = [phi, -theta, 0, -pi/2, 0, 0]

    backend.step()



#robot.plot(robot.q, block=True)