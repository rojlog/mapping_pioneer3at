#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from mappping_zmq import Pioneer3AT

class MapBuilder_Dynamic:
    def __init__(self):
        rospy.init_node('occupancy_grid_publisher')

        # Inicializar el mapa de ocupación como un array de NumPy lleno de -1, que indica áreas no exploradas.
        self.map_size_x = 100  # en celdas
        self.map_size_y = 100  # en celdas
        self.map_resolution = 0.1  # metros por celda
        self.occupancy_grid = np.full((self.map_size_x, self.map_size_y), -1, dtype=int)

        # Configurar publicadores y suscriptores
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.laser_sub = rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)

        # Configurar el mensaje OccupancyGrid, guardamos la info occupancy-grid en un mensaje de ROS
        self.map_msg = OccupancyGrid()
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = self.map_size_x
        self.map_msg.info.height = self.map_size_y

        # Establecer el origen del mapa a alguna posición, normalmente la posición inicial del robot    
        self.map_msg.info.origin.position.x = -self.map_size_x * self.map_resolution / 2         # xr = 50 + m.ceil(xw/0.1)   conversion del doc
        self.map_msg.info.origin.position.y = -self.map_size_y * self.map_resolution / 2         # yr = 50 - m.floor(yw/0.1)  posicion como tipo imagen y_max, x_min
        self.map_msg.info.origin.orientation.w = 1

    # Actualizamos la posicion del robot en el mapa
    def update_map(self, Pioneer3AT):
        Pioneer3AT.update_position_orientation_gz()
        x_r_gz = Pioneer3AT.pioneer_3at['position_gz']['x']   # Posicion en x del robot en el mundo gazebo
        y_r_gz = Pioneer3AT.pioneer_3at['position_gz']['y']   # Posicion en y del robot en el mundo gazebo
        print("x_r_gz: ", x_r_gz)





    def laser_callback(self, data):
        # Aquí procesarías los datos del láser y actualizarías self.occupancy_grid
        # Por ejemplo, podrías realizar un algoritmo de ray-casting o alguna forma de sensor fusion.

        # Convertir el mapa de ocupación de NumPy a un formato que pueda ser enviado por ROS
        self.map_msg.data = self.occupancy_grid.ravel().tolist()

        # Publicar el mapa de ocupación
        self.map_pub.publish(self.map_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    map_builder = MapBuilder_Dynamic()
    map_builder.update_map()
    #map_builder.run()
