import roboticstoolbox as rtb
import numpy as np

from numpy import pi


if __name__ == "__main__":
    
    ############# DEFINIR EL ROBOT QUE SE QUIERA USAR ################


    robot_UR5e = rtb.roboticstoolbox.models.UR5()

    home_p = [0.30024131, 0.29984787, 0.15009143, -2.98728282, 1.53174465, 1.51506493]  # home cartesianas UR5e.
    home_q = [0.518,-1.476,2.278,-2.394,-1.538,-3.984]                                  # home articulares UR5e.

    


    robot_UR3 = rtb.DHRobot(
    [
        rtb.RevoluteDH(d=0.1519, alpha=pi/2, qlim=np.deg2rad([-363, 363])),     # Base
        rtb.RevoluteDH(a=-0.24365, qlim=np.deg2rad([-363, 363])),               # Hombro (realmente de -364 a 363)
        rtb.RevoluteDH(a=-0.21325, qlim=np.deg2rad([-363, 363])),               # Codo
        rtb.RevoluteDH(d=0.11235, alpha=pi/2, qlim=np.deg2rad([-363, 363])),    # Muñeca 1
        rtb.RevoluteDH(d=0.08535, alpha=-pi/2, qlim=np.deg2rad([-363, 363])),   # Muñeca 2
        rtb.RevoluteDH(d=0.0819, qlim=np.deg2rad([-1000, 1000]))                # Muñeca 3 (sin límites articuares)
    ], name="UR3")

    home_p = [-0.11841544, -0.26807058, 0.15727483, 3.11652424, 0.02504716, -3.14082373]    # home cartesianas UR3.
    home_q = np.deg2rad([-91.71, -98.96, -126.22, -46.29, 91.39, 358.21])                   # home articulares UR3.
    
    ###############################################################
    

    robot_model = robot_UR3 

    print(robot_model)

    if isinstance(robot_model, rtb.ERobot) and robot_model.hasgeometry:
        from interfaces.app_swift import app_swift as app
    else:
        from interfaces.app_pyplot import app_pyplot as app


    print("[#] Elija el modo de la interfaz hombre-máquina.")
    print("\t-> 1. Modo Base: Muestra la simulación de la interfaz.")
    print("\t-> 2. Modo Gazebo: Controla una simulación en Gazebo del robot.")
    print("\t-> 3. Modo Real: Controla un robot real.\n")

    mode = int(input("Mode: "))

    if mode == 1:
        from simuladores.simulation_base import Simulation

    if mode == 2:
        import rosgraph
        if not rosgraph.is_master_online():
            exit("[!]: No se detecta el master.")
        from simuladores.simulation_gazebo import Simulation
        import rospy
        rospy.init_node("arm_controller")
    if mode == 3:
        from simuladores.simulation_demo import Simulation
        

    window = app(Simulation, robot_model, home_q)

    window.run()