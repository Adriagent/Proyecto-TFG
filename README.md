# Interfaz hombre-máquina mediante estimación de pose
<p align="center">
  <img src="doc/Pipeline.jpg" alt="animated"/>
</p>

## Índice:
  
  - [1. Introducción](#p1)
  - [2. Instalación](#p2)  
  - [2.1. Simulated Environment](#p2.1) 
  - [2.2. Real Environment](#p2.2)  
  - [3. Video](#p3)  
  - [4. Creditos](#p4)  

## Introducción: <a name="p1"/>

Este es el proyecto de mi trabajo de fin de grado en la carrera de Ingeniería Robótica, en la Universidad de Alicante. El proyecto se ha desarrollado en `Ubuntu 20.4` y completamente en `Python 3`. La interfaz permite controlar una simulación en el entorno `Gazebo` de un robot UR y también controlar un robot real.
deEn este proyecto se ha desarrollado una intefaz hombre-máquina para teleoperar brazos robots a partir de gestos.

## Instalación: <a name="p2"/>

Las dependencias necesarias para que funcione el proyecto se pueden instalar automáticamente a partir del fichero `requirements.txt` mediante el siguiente comando.

    $ pip3 install -r requirements.txt

Por otra parte, si se quiere utilizar la interfaz para controlar un robot en Gazebo, es necesario descargarse el [paquete de Universal Robots](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).
 
 Simulated environment has been coded with `Python3` on `ROS Noetic`. First of all, open Gazebo simulator, in this case, ROS defalut house map is used. On one terminal execute following commands:

    $ export TURTLEBOT3_MODEL=waffle
    $ roslaunch turtlebot3_gazebo turtlebot3_house.launch

  Open new terminal and execute navigation module (is is necessary to have previosly mapped the area):
  
    $ export TURTLEBOT3_MODEL=waffle
    $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/house.yaml
  _Note: Both files (`.yaml` and `.pgm`) must be on `HOME` directory._
   
  This will open a rviz window where robot, map and localization point cloud can be seen. Use 2D POSE ESTIMATE to estimate the current position of the robot in the map.
  
  Execute code `main.py` and open `Andorid app`. TCP conexion uses `port:12343`

## Video: <a name="p3"/>

Click on the gif to see a video of the project in action:

<p align="center">
  <a href="[https://youtu.be/j-LswYOt--s](https://www.youtube.com/watch?v=e_8cTOLwNLA)">
    <img src="doc/clip.gif" alt="animated"/>
  </a>
</p>

## Credits: <a name="p4"/>

Este proyecto es un trabajo de final de grado hecho por Adrián Sanchis Reig
