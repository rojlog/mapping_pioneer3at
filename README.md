# mapping_pioneer3at

mapping_pioneer3at This repository contains a mapping simulation with the Pioneer 3-AT mobile robot using the Gmapping package. Much of the Pioneer 3-AT assembly code is based on the repository available at https://github.com/Gastd/p3at_tutorial.

This repository contains a mapping simulation with the Pioneer 3-AT mobile robot using the Gmapping package. Much of the Pioneer 3-AT assembly code is based on the repository available at https://github.com/Gastd/p3at_tutorial.


# Pioneer 3-AT Mapping Simulation

Este proyecto utiliza el simulador multi-robot Gazebo y el Sistema Operativo Robótico (ROS) para realizar mapeo con el robot Pioneer 3-AT. La simulación se ejecuta en Ubuntu 20.04 con ROS Noetic y Gazebo versión 11.14.0.

## Requisitos del sistema

- Ubuntu 20.04
- ROS Noetic
- Gazebo 11.14.0
- Python 3.8

## Dependencias

Este proyecto requiere las siguientes bibliotecas de Python y paquetes de ROS:

- `rospy`
- `tf2_ros`
- `rospkg`
- `tf.transformations`
- `scipy.interpolate`
- `matplotlib.pyplot`
- `random`
- `numpy`
- `math`
- `shapely.geometry`
- `geometry_msgs.msg`
- `nav_msgs.msg`
- `os`
- `json`

## Instalación

Para compilar el proyecto, sigue estos pasos:

1. Clona el repositorio en el directorio `src` de tu espacio de trabajo ROS:
   ```bash
   cd ~/catkin_ws/src
   git clone [URL_DEL_REPOSITORIO]
