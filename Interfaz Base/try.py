import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3

home_p = [0.11841544, 0.26807058, 0.15727483, 2.35307635, 1.53548811, -2.35822761] # home cartesianas
home_q = np.deg2rad([-91.71, -98.96, -126.22, -46.29, 91.39, 358.21])

robot = rtb.roboticstoolbox.models.UR3()
robot.q = home_q
desired_q = robot.q # Inicializamos la posición deseada con la actual del robot.
robot.base = SE3(0,0,0)



##

T = robot.fkine(robot.q, end=robot.ee_links[0])