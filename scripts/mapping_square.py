#!/usr/bin/env python3

import rospy
import tf2_ros
import rospkg
import tf.transformations

import scipy.interpolate as spi
import matplotlib.pyplot as plt
import random as rd
import numpy as np
import math as mt

import os
import json

from shapely.geometry import LineString

import astartmod as astart

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid


class Pioneer3AT():
    def __init__(self):
        
        # twist()
        self.move_cmd = Twist()

        # linear velocity
        self.linear_speed = 0.0
        
        self.Kv = 0.2
        self.Kh = 1.2


        # angular velocity and move
        self.angular_speed = 0.0 # radians for second
        
        # publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        
        # time sleep one second
        self.move_time = 10.0
        
        self.duration = rospy.Duration(15.0)

        # Map
        self.map_data = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Save data
        self.file_path = None

        self.data = {
            'data': []
        }


        self.random_position = {
            'postion': [(59, 243), (59, 563), (179, 563), (179, 243)],
            'choices': [0, 1, 2, 3],
            'choices_restore': [0, 1, 2, 3],
            'last_number': -1
        }


        self.PID_control = {
            'Kp_linear': 0.1,
            'Kd_linear': 0.3,
            'Ki_linear': 0.0,			
            'derivative_error_linear':0.0,
            'integral_error_linear': 0.0,
            
            
            'Kp_angular': 1.0,
            'Kd_angular': 0.05,
            'Ki_angular': 0.0,
            'derivative_error_angular':0.0,
            'integral_error_angular': 0.0
        }


        self.trajectory = {
            'a_start_x': [],
            'a_start_y': [],
            'trajectory': [],
            'path_a_start': [],
            'new_trajectory': []
        }


        self.pioneer_3at = {
            'position_gz': {'x': 0, 'y': 0},  # coordenates in gazebo NECESITAMOS TRANSFORMALOS A PUNTOS NO OCUPAN SER UNA LISTA
            'orientation_gz': {'yaw': 0.0},
            'position_grid': {'x': 0, 'y': 0},  # coordenates in occupancy grid
            'orientation_grid': {'yaw': 0},
            # 'parent_frame': 'map',
            'parent_frame': 'odom',
            'child_frame': 'base_link',
            'goal': {'goal': False}
        }

        # robot coonfiguration position with tf2_ros
        self.tf_Buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_Buffer)


    def update_position_orientation_gz(self):
        self.robot_gazebo_position()
        self.robot_gazebo_orientation()


    def update_position_orientation_grid(self):
        self.update_position_orientation_gz()
        resolution = self.map_data.info.resolution
        odom_origin = (self.map_data.info.origin.position.x, self.map_data.info.origin.position.y)
        #print(self.map_data.info.data[-1][-20])
        #print('ODOM = ', odom_origin)
        self.pioneer_3at['position_grid']['x'] = int((self.pioneer_3at['position_gz']['x'] - odom_origin[0]) / resolution)
        self.pioneer_3at['position_grid']['y'] = int((self.pioneer_3at['position_gz']['y'] - odom_origin[1]) / resolution)
    def robot_gazebo_position(self):
        try:
            trans = self.tf_Buffer.lookup_transform(self.pioneer_3at['parent_frame'], self.pioneer_3at['child_frame'], rospy.Time(0), rospy.Duration(1.0))
            self.pioneer_3at['position_gz']['x'] = trans.transform.translation.x
            self.pioneer_3at['position_gz']['y'] = trans.transform.translation.y
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Robot transofrmation traslation error: %s " % str(e))
            return None
    def robot_gazebo_orientation(self):
        try:
            trans = self.tf_Buffer.lookup_transform(self.pioneer_3at['parent_frame'], self.pioneer_3at['child_frame'], rospy.Time(0), rospy.Duration(1.0))
            quaternion = (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.pioneer_3at['orientation_gz']['yaw'] = euler[2] # np.pi/4
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Robot transofrmation orientation error: %s " % str(e))
            return None
            
    def map_callback(self, msg):
        self.map_data = msg

    # save the occupancy grid in a matrix 
    def grid_matrix(self):
        while self.map_data is None and not rospy.is_shutdown():
            print("Waiting for the map data to become available...")
            rospy.sleep(1)  # Esperamos 1 seg
            
        grid_matrix = np.array(self.map_data.data, dtype='int8')

        width = self.map_data.info.width
        height = self.map_data.info.height

        grid_matrix = grid_matrix.reshape((height, width))
        return grid_matrix


    def a_start(self):
        new_map = self.grid_matrix()
        new_map[new_map == -1] = 0
        new_map = new_map.T

        self.update_position_orientation_gz()	
        self.update_position_orientation_grid()
    
        # Si es mayor a uno existen puntos en los que puede trazar una trayectoria
        if len(self.random_position['choices']) > 1:
            self.random_position['last_number'] = rd.choice(self.random_position['choices'])
            self.random_position['choices'].remove(self.random_position['last_number'])
        else: # Si choices solo queda le asignamos ese valor y reestablcemos las posiciones eliminando la posicion donde se encuentra 
            self.random_position['last_number'] = self.random_position['choices'][0]
            self.random_position['choices'] = self.random_position['choices_restore'].copy()
            self.random_position['choices'].remove(self.random_position['last_number'])

        self.trajectory['path_a_start'] = astart.astar(new_map, (self.pioneer_3at['position_grid']['x'], self.pioneer_3at['position_grid']['y']), 
                      self.random_position['postion'][self.random_position['last_number']], allow_diagonal_movement=True)
        
        return self.trajectory['path_a_start']

    def angdiff(self, t2):
        """
        Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
        """		
        t1 = self.pioneer_3at['orientation_gz']['yaw']
        # The angle magnitude comes from the dot product of two vectors
        angmag = mt.acos(mt.cos(t1)*mt.cos(t2)+mt.sin(t1)*mt.sin(t2))
        #print(angmag)
        # The direction of rotation comes from the sign of the cross product of two vectors
        angdir = mt.cos(t1)*mt.sin(t2)-mt.sin(t1)*mt.cos(t2)
        return mt.copysign(angmag, angdir)
    
    def recopiling_data(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mapping_pioneer3at')  
        
        # Construye la ruta al archivo dentro del directorio 'data' del paquete
        self.file_path = os.path.join(package_path, 'data', 'data_file.json')

        # self.plot_path = os.path.join(package_path, 'graphics', 'plot_file.png')
        self.plot_path = os.path.join(package_path, 'graphics/')


        # Revisar de que el directorio 'data' existe
        if not os.path.exists(os.path.dirname(self.file_path)):
            os.makedirs(os.path.dirname(self.file_path))	

        if not os.path.exists(os.path.dirname(self.plot_path)):
            os.makedirs(os.path.dirname(self.plot_path))	

        if self.file_path:
            print('Path data exists')

        if self.plot_path:
            print('Path plot exists')


    def trajectory_robot(self):

        # Crea una LineString con los puntos
        line = LineString(self.a_start())
        epsilon = 0.3
        # Aplica el algoritmo Ramer-Douglas-Peucker con un umbral de epsilon
        simplified_line = line.simplify(epsilon, preserve_topology=False)

        # Obtén los puntos simplificados
        #simplified_points = list(simplified_line.coords)

        simplified_points = self.a_start()

        xarr = np.array([i[0] for i in simplified_points])
        print(xarr)
        print('---------------------------------')
        yarr = np.array([i[1] for i in simplified_points])
        print(yarr)

        duration_sec = self.duration.to_sec()

        tarr = np.linspace(0, duration_sec, xarr.shape[0])

        if xarr.shape[0] == 3:
            xc = spi.splrep(tarr, xarr, k=2, s=0)
            yc = spi.splrep(tarr, yarr, k=2, s=0)		
        elif xarr.shape[0] == 2:
            xc = spi.splrep(tarr, xarr, k=1, s=0)
            yc = spi.splrep(tarr, yarr, k=1, s=0)
        else:
            xc = spi.splrep(tarr, xarr, s=0)
            yc = spi.splrep(tarr, yarr, s=0)


        init_time = rospy.Time.now().to_sec()
        


        ruta_xr = []
        ruta_yr = []
        linear_velocity = []

        ruta_xd = []
        ruta_yd = []
        time_d = []
        self.recopiling_data()
        init_time = rospy.Time.now().to_sec()
        time_now = rospy.Time.now().to_sec()
        previous_error_linear = 0.0
        previous_error_angular = 0.0

        errp = 0.0
        errh = 0.0

        while time_now - init_time < duration_sec:
            time_now = rospy.Time.now().to_sec() 
            
            self.update_position_orientation_gz()
            self.update_position_orientation_grid()
            dt = time_now - init_time

            # COnvercion a float ya que spi devuelve npdarray
            xd = spi.splev(dt, xc, der=0)
            yd = spi.splev(dt, yc, der=0)

            xd = float(xd)
            yd = float(yd)


            ruta_xd.append(xd)
            ruta_yd.append(yd)
            ruta_xr.append(self.pioneer_3at['position_grid']['x'])
            ruta_yr.append(self.pioneer_3at['position_grid']['y'])
            time_d.append(dt)

            # Calculate the current error
            error_x = xd - self.pioneer_3at['position_grid']['x']
            error_y = yd - self.pioneer_3at['position_grid']['y']
            errp = mt.sqrt(error_x**2 + error_y**2)
            
            angd = mt.atan2(yd - self.pioneer_3at['position_grid']['y'], 
                            xd - self.pioneer_3at['position_grid']['x'])
            errh = self.angdiff(angd)

            # PID calculations
            # self.PID_control['integral_error_linear'] += errp * dt
            #self.PID_control['integral_error_angular'] += errh * dt
            
            self.PID_control['derivative_error_linear'] = (errp - previous_error_linear) / dt if dt > 0 else 0 #Derivada
            self.PID_control['derivative_error_angular'] = (errh - previous_error_angular) / dt if dt > 0 else 0
            
            self.linear_speed =  self.PID_control['Kp_linear'] * errp  + self.PID_control['Kd_linear']  * self.PID_control['derivative_error_linear']  # + self.PID_control['Ki_linear'] * self.PID_control['integral_error_linear']
            self.angular_speed = self.PID_control['Kp_angular'] * errh + self.PID_control['Kd_angular'] * self.PID_control['derivative_error_angular'] # + self.PID_control['Ki_angular'] * self.PID_control['integral_error_angular']
            
            linear_velocity.append(self.linear_speed)
            # self.linear_speed = self.Kv * errp*0.05 # NO BORRAR --------------------------->
            # self.angular_speed = self.Kh * errh     # NO BORRAR --------------------------->
            
            previous_error_linear = errp
            previous_error_angular = errh

            self.move_robot()


            data_point = {
                'time': dt,
                'xd': xd,
                'yd': yd,
                'xr': self.pioneer_3at['position_grid']['x'],
                'yr': self.pioneer_3at['position_grid']['y'],
                'linear_velocity': self.linear_speed,
                'angular_velocity': self.angular_speed
            }

            self.data['data'].append(data_point)

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()

        mae = np.mean(np.abs(np.array(ruta_xd) - np.array(ruta_xr)) + np.abs(np.array(ruta_yd) - np.array(ruta_yr)))
        print(f"Mean Absolute Error: {mae}")

        fig, axs = plt.subplots(2, 2, figsize=(10, 8))

        # Gráfico 1: Seno
        axs[0, 0].plot(ruta_xd, ruta_yd)
        axs[0, 0].set_title('path (xd, yd)')

        # Gráfico 2: Coseno
        axs[0, 1].plot(time_d, ruta_xd)
        axs[0, 1].set_title('time vs xd')

        # Gráfico 3: Tangente
        axs[1, 0].plot(time_d, ruta_xd)
        axs[1, 0].set_title('time vs yd')

        # Gráfico 4: Exponencial negativa
        axs[1, 1].plot(time_d, linear_velocity)
        axs[1, 1].set_title('Time vs linear velocity')

        # Ajustar layout para evitar la superposición
        plt.tight_layout()

        plt.savefig(self.plot_path + '_2')


        # Crear la gráfica
        plt.figure(figsize=(10, 5))

        # Graficar xd, yd para la ruta deseada
        plt.plot(ruta_xd, ruta_yd, label='Ruta deseada (xd, yd)', color='blue', marker='o')

        # Graficar xr, yr para la posición real del robot
        plt.plot(ruta_xr, ruta_yr, label='Posición real del robot (xr, yr)', color='red', marker='x')

        # Añadir leyenda para identificar cada conjunto de datos
        plt.legend()

        # Añadir títulos y etiquetas para los ejes
        plt.title('Comparación de Ruta Deseada y Posición Real del Robot')
        plt.xlabel('Coordenada X')
        plt.ylabel('Coordenada Y')

        # Guardar la gráfica en un archivo
        plt.savefig(self.plot_path + '_1')


        # save data in a JSON file	
        with open(self.file_path, 'w') as json_file:
            json.dump(self.data, json_file, indent=4)

        


    def move_robot(self):
        self.move_cmd.linear.x = self.linear_speed
        # print(self.move_cmd.linear.x)
        self.move_cmd.angular.z = self.angular_speed
        self.cmd_pub.publish(self.move_cmd)


# if es para poder usarlo como modulo
if __name__ == '__main__':                          
    rospy.init_node('Pioneer3AT')

    P3AT = Pioneer3AT()

    try:
        rate = rospy.Rate(10)
        #P3AT.next_position_gz()
        #P3AT.next_grid_position()
        #P3AT.minimises_entropy()
        #P3AT.a_start()
        #while not rospy.is_shutdown():
        P3AT.trajectory_robot()
        #P3AT.update_position_orientation_grid()
        rate.sleep()
    except rospy.ROSInterruptException:
        pass


    
        
"""
queue_size = 10 ---> si tiene cola de 10, si llega a fallar el sistema guarda los ultimos 10 msj 
"""