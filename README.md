# Interfaz hombre-máquina mediante estimación de pose
<p align="center">
  <img src="doc/Pipeline.jpg" alt="animated"/>
</p>

## Índice:
  
  - [1. Introducción](#p1)
  - [2. Instalación](#p2)  
  - [3. Ejemplos de uso](#p3)
  - [4. Video](#p4)
  - [5. Creditos](#p5)  

## Introducción: <a name="p1"/>

Este es el proyecto de mi trabajo de fin de grado en la carrera de Ingeniería Robótica, en la Universidad de Alicante. El proyecto se ha desarrollado en `Ubuntu 20.04.4` y programando en Python 3. La interfaz permite controlar una simulación en el entorno Gazebo de un robot UR y también controlar un robot real.

## Instalación: <a name="p2"/>

Las dependencias necesarias para que funcione el proyecto se pueden instalar automáticamente a partir del fichero `requirements.txt` mediante el siguiente comando.

    $ pip3 install -r requirements.txt

Por otra parte, si se quiere utilizar la interfaz para controlar un robot en Gazebo, es necesario instalar el [paquete](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) de Universal Robots para ROS. La conexión entre Gazebo y la interfaz se ha hecho en `ROS Noetic`.

    # source global ros
    $ source /opt/ros/<your_ros_version>/setup.bash
    
    # create a catkin workspace
    $ mkdir -p catkin_ws/src && cd catkin_ws
    
    # clone the driver
    $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
    
    # clone fork of the description. This is currently necessary, until the changes are merged upstream.
    $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
    
    # install dependencies
    $ sudo apt update -qq
    $ rosdep update
    $ rosdep install --from-paths src --ignore-src -y
    
    # build the workspace
    $ catkin_make
    
    # activate the workspace (ie: source it)
    $ source devel/setup.bash

## Ejemplos de uso: <a name="p3"/>

Para iniciar el proyecto, es necesario haber realizado la calibración de las cámaras y del estéreo en primer lugar. Para ello hace falta utilizar un tablero de 10x7 cuadrados. En la siguiente página se puede configurar a gusto el tablero que queramos: [Generate Your Own Checkerboards](https://markhedleyjones.com/projects/calibration-checkerboard-collection).

<p align="center">
  <img height=400 scale="60" src="doc/board.jpg"/>
</p>

Figura A.1: Tablero 9x6 aristas / 10x7 cuadrados para calibrar las cámaras.
La calibración de las cámaras se lleva a cabo ejecutando el código camera_calibration.py,
tras realizarla, se puede calibrar el estéreo mediante el código stereo_calibration.py. Tras realizar estos dos pasos, ya es posible comenzar a utilizar el proyecto.
Para iniciar la interfaz, hay que ejecutar el fichero main.py, el cual permite seleccionar el
modo en el que se quiere utilizar la interfaz. En el modo base solo se puede controlar la representación del robot integrada en la interfaz. El modo Gazebo, que necesita que previamente
se haya lanzado el launch del controlador, permite controlar tanto la representación del robot como una simulación de este en Gazebo. Por último, el modo demo permite controlar un
brazo robot en tiempo real iniciando un servidor.
Si se quiere iniciar el modo Gazebo, previamente hay que lanzar el launch del controlador
del robot que queramos controlar. El comando para hacerlo es el siguiente.
1 $ roslaunch ur_gazebo UR<modelo>_bringup.launch
Por otra parte, si se quiere controlar el robot real, la demo está preparada para comunicarse
con un UR3. Dentro de la carpeta del proyecto se encuentra el fichero ur3_control.urp. Este
es el programa que debe ejecutarse desde el robot real para que pueda comunicarse con la
aplicación y moverse.
En caso de que no se disponga de dos cámaras, el proyecto cuenta con dos vídeos y la
calibración de las cámaras y el estéreo que se utilizó para obtenerlos. Por lo que se puede
ejecutar la interfaz sin necesidad de disponer de ninguna cámara ni de realizar la calibración.
Para lanzar el proyecto en este modo, es necesario ejecutar el main con el argumento ”-vid”:
python3 main.py -vid.

## Vídeo: <a name="p4"/>

Demostración probando la interfaz con un robot UR3: [vídeo](https://www.youtube.com/watch?v=e_8cTOLwNLA)

<p align="center">
  <a href="https://www.youtube.com/watch?v=e_8cTOLwNLA">
    <img src="doc/clip.gif"/>
  </a>
</p>

## Créditos: <a name="p5"/>

Este proyecto es un trabajo de final de grado hecho por Adrián Sanchis Reig
