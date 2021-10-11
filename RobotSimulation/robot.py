import roboticstoolbox as rtb
from roboticstoolbox.backends.Swift import Swift
from spatialmath import SE3
from math import pi

robot = rtb.roboticstoolbox.models.UR5()


# Obtenemos la matriz de transformacion:
T = SE3(0,0,0.5)*SE3.RPY((0,0,0))
robot.base = T

print(robot)

robot.q = [0, 0, 0, -pi/2, 0, 0]

print(robot.q)

backend = Swift()
backend.launch()
backend.add(robot)

while True:
    None


#robot.plot(robot.q, block=True)
