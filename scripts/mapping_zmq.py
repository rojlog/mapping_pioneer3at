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
import cv2 as cv

import os
import json

from shapely.geometry import LineString

import astartmod as astart

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

class Pioneer3AT():
    def __init__(self):
        
        # twist()
        self.move_cmd = Twist()

        # linear velocity
        self.linear_speed = 0.0
        
        self.Kv = 0.2
        self.Kh = 1.2

###### Variables del MAPA ######

        # self.xr_gz = self.p3at.pioneer_3at['position_gz']['x']   # Posicion en x del robot en el mundo gazebo
        # self.yr_gz = self.p3at.pioneer_3at['position_gz']['y']   # Posicion en y del robot en el mundo gazebo
        # print("**************** x_r_gz: ", self.xr_gz)
        # print("**************** y_r_gz: ", self.yr_gz)

        # Fijamos la posición de odom a donde inicio el robot en el mundo gazebo
        self.ODOM_ORIGIN = (0, 0)  # Posición de origen del mapa de ocupación en el mundo gazebo, esquina superior izquierda, despues de invertir el eje y


        # Inicializar el mapa de ocupación como un array de NumPy lleno de -1, que indica áreas no exploradas.
        self.map_size_x = 500  # en celdas
        self.map_size_y = 500  # en celdas
        self.map_resolution = 0.1  # metros por celda
        
        #self.occupancy = np.full((self.map_size_x, self.map_size_y), -1, dtype=int)
        #self.occupancy_grid = np.uint8(self.occupancy[::-1]) # Invertir el mapa (matriz) para que se maneje como coordenadas de imagen

        # casteo a np.unit8 para que sea compatible con OpenCV        
        # self.occupancy_grid = np.uint8(np.full((self.map_size_x, self.map_size_y), -1, dtype='int8'))
        self.occupancy_grid = 0.5*np.ones((self.map_size_x, self.map_size_y))
        self.tocc = np.zeros((self.map_size_x, self.map_size_y))
        #self.tocc = np.uint8(np.zeros((self.map_size_x, self.map_size_y), dtype='int8'))
        # Configurar publicadores y suscriptores
        #self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.laser_sub = rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)

        # Configurar el mensaje OccupancyGrid, guardamos la info occupancy-grid en un mensaje de ROS
        self.map_msg = OccupancyGrid()
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = self.map_size_x
        self.map_msg.info.height = self.map_size_y

        # Establecer el origen del mapa a alguna posición, usamos la posición inicial del robot    
        self.map_msg.info.origin.position.x = int((self.ODOM_ORIGIN[0]) / self.map_msg.info.resolution)         # xr = 50 + m.ceil(xw/0.1)   Conversion del doc
        self.map_msg.info.origin.position.y = int((self.ODOM_ORIGIN[1]) / self.map_msg.info.resolution)         # yr = 50 - m.floor(yw/0.1)  Posicion como tipo imagen y_max, x_min
        #self.map_msg.info.origin.orientation.w = 1

        # Converción posición del robot de Gazebo a OccupancyGrid
        # self.xr_grid = int((self.xr_gz - self.ODOM_ORIGIN[0]) / self.map_msg.info.resolution) 
        # self.yr_grid = int((self.yr_gz - self.ODOM_ORIGIN[1]) / self.map_msg.info.resolution) + 250

################################################################################################################################




        # angular velocity and move
        self.angular_speed = 0.0 # radians for second
        
        # publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        
        # time sleep one second
        self.move_time = 10.0
        
        self.duration = rospy.Duration(30.0)

        # Map
        self.map_data = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # Save data
        self.file_path = None

        self.data = {
            'data': []
        }


        self.random_position = {
            # 'postion': [(59, 243), (59, 563), (179, 563), (179, 243)],
            'position_map_builder': [(20, 330), (80, 330), (80, 180), (20, 180)],
            # 'position_map_builder': [(20, 330), (80, 330)],
            'choices': [0, 1, 2, 3],
            # 'choices': [0, 1],
            'choices_restore': [0, 1, 2, 3],
            # 'choices_restore': [0, 1],
            'last_number': -1
        }


        self.PID_control = {
            'Kp_linear': 0.15,
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


    def update_position_grid(self):
        self.update_position_orientation_gz()
        resolution = self.map_msg.info.resolution
        odom_origin = self.ODOM_ORIGIN

        self.pioneer_3at['position_grid']['x'] = int((self.pioneer_3at['position_gz']['x'] - odom_origin[0]) / resolution)
        self.pioneer_3at['position_grid']['y'] = int((self.pioneer_3at['position_gz']['y'] - odom_origin[1]) / resolution) + 250


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
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        # new_map = self.grid_matrix()
        new_map = self.occupancy_grid.copy()
        # new_map[new_map == -1] = 0
        new_map[new_map != -1] = 0
        new_map = new_map.T

        self.update_position_orientation_gz()	
        self.update_position_grid()
    
        # Si es mayor a uno existen puntos en los que puede trazar una trayectoria, estos solo son numeros 0, 1, 2, 3
        if len(self.random_position['choices']) > 1:
            self.random_position['last_number'] = rd.choice(self.random_position['choices'])
            self.random_position['choices'].remove(self.random_position['last_number'])
        else: # Si choices solo queda le asignamos ese valor y reestablcemos las posiciones eliminando la posicion donde se encuentra 
            self.random_position['last_number'] = self.random_position['choices'][0]
            print('Ultimo posición = ', self.random_position['last_number'])
            self.random_position['choices'] = self.random_position['choices_restore'].copy()
            print('Restaurando posiciones = ', self.random_position['choices'])
            self.random_position['choices'].remove(self.random_position['last_number'])
            print('Despues de eliminar el ultimo = ', self.random_position['choices'])

        self.trajectory['path_a_start'] = astart.astar(new_map, (self.pioneer_3at['position_grid']['x'], self.pioneer_3at['position_grid']['y']), 
                      self.random_position['position_map_builder'][self.random_position['last_number']], allow_diagonal_movement=True)
        
        for i in range(len(self.trajectory['path_a_start'])):
            new_map[self.trajectory['path_a_start'][i][1]][self.trajectory['path_a_start'][i][0]] = 100
        
        # print(self.trajectory['path_a_start'])
        # plt.imshow(new_map)
        # plt.show()  # Mostrar la imagen del mapa de ocupación
        print('Desde ', self.pioneer_3at['position_grid']['x'], self.pioneer_3at['position_grid']['y'], ' Hasta ', self.trajectory['path_a_start'][-1]) 
        plt.imshow(self.occupancy_grid)
        plt.show()  # Mostrar la imagen del mapa de ocupación
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

        while not rospy.is_shutdown():
            # Crea una LineString con los puntos
            line = LineString(self.a_start())
            epsilon = 0.3
            # Aplica el algoritmo Ramer-Douglas-Peucker con un umbral de epsilon
            simplified_line = line.simplify(epsilon, preserve_topology=False)

            # Obtén los puntos simplificados
            simplified_points = list(simplified_line.coords)
            # simplified_points = self.a_start()
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

            # errp = 100

            while time_now - init_time < duration_sec:
            # while errp > 10.0:
                time_now = rospy.Time.now().to_sec() 
                
                self.update_position_orientation_gz()
                self.update_position_grid()
                dt = time_now - init_time

                # COnvercion a float ya que spi devuelve npdarray
                xdd = spi.splev(dt, xc, der=0)
                ydd = spi.splev(dt, yc, der=0)

                xd = float(xdd)
                yd = float(ydd)


                ruta_xd.append(xd)
                ruta_yd.append(yd)
                ruta_xr.append(self.pioneer_3at['position_grid']['x'])
                ruta_yr.append(self.pioneer_3at['position_grid']['y'])
                time_d.append(dt)

                # Calculate the current error
                error_x = xd - self.pioneer_3at['position_grid']['x']
                error_y = yd - self.pioneer_3at['position_grid']['y']
                # error_x = xarr[-1] - self.pioneer_3at['position_grid']['x']
                # error_y = yarr[-1] - self.pioneer_3at['position_grid']['y']
                errp = mt.sqrt(error_x**2 + error_y**2)

                angd = mt.atan2(yd - self.pioneer_3at['position_grid']['y'], 
                                xd - self.pioneer_3at['position_grid']['x'])
                # angd = mt.atan2(yarr[-1] - self.pioneer_3at['position_grid']['y'],
                #                 xarr[-1] - self.pioneer_3at['position_grid']['x'])
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
                
                
                self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
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



    #######################################################################################################
    #######################################################################################################
    #######################################################################################################
    

    def laser_callback(self, msg):

        # Asumiendo que el robot está en el origen (0,0) para este ejemplo
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        coordinates = []
        # print('Cantidad dee rayos ',len(msg.ranges))
        
        for i, distance in enumerate(msg.ranges): # obtenemos la distancia a la que se encuentra el objeto
            # Ignorar medidas sin retorno o fuera de los límites definidos
        
            """
            comprueba si la distancia medida no es infinita. math.isinf() verifica si un número es infinito, 
            lo cual puede ser utilizado por algunos sensores láser para indicar que no se detectó ningún objeto en esa dirección 
            (es decir, el láser no "rebotó" en nada dentro de su rango máximo). Si la distancia es infinita, se ignora la medida.
            """
            
            if not mt.isinf(distance):
                yaw_robot = self.pioneer_3at['orientation_gz']['yaw'] # = angle_min + i * angle_increment
                x_robot = self.pioneer_3at['position_gz']['x']
                y_robot = self.pioneer_3at['position_gz']['y']
                theta = angle_min + i * angle_increment
                x = distance * mt.cos(theta)
                y = distance * mt.sin(theta)

                # Calcular la posición relativa al robot.
                x_rel = distance * mt.cos(theta)
                y_rel = distance * mt.sin(theta)


                # Transformar la posición relativa al marco de referencia global.
                x_world = x_robot + (x_rel * mt.cos(yaw_robot) - y_rel * mt.sin(yaw_robot))
                y_world = y_robot + (x_rel * mt.sin(yaw_robot) + y_rel * mt.cos(yaw_robot))

                x_pos = int((x_world - self.ODOM_ORIGIN[0]) / self.map_msg.info.resolution) 
                y_pos = int((y_world - self.ODOM_ORIGIN[1]) / self.map_msg.info.resolution) + 250
                coordinates.append((x_pos, y_pos))


        for i in range(len(coordinates)):
            cv.line(self.occupancy_grid, (self.pioneer_3at['position_grid']['x'], self.pioneer_3at['position_grid']['y']), coordinates[i], (0,0,0), 1)
            self.tocc[coordinates[i][1]][coordinates[i][0]] = 1

        self.occupancy_grid[self.pioneer_3at['position_grid']['y']][self.pioneer_3at['position_grid']['x']]= 100  # posicion del robot

        # plt.imshow(self.occupancy_grid + self.tocc)
        # plt.show()  # Mostrar la imagen del mapa de ocupación



##########################################################################################################
##########################################################################################################
##########################################################################################################
    






# if es para poder usarlo como modulo
if __name__ == '__main__':                          
    rospy.init_node('Pioneer3AT')

    P3AT = Pioneer3AT()
    #Map = MapBuilder_Dynamic(P3AT)
    try:
        rate = rospy.Rate(10)
        #P3AT.next_position_gz()
        #P3AT.next_grid_position()
        #P3AT.minimises_entropy()
        #P3AT.a_start()
        #while not rospy.is_shutdown():
        #P3AT.trajectory_robot()
        # P3AT.a_start()
        P3AT.trajectory_robot()

        rate.sleep()
    except rospy.ROSInterruptException:
        pass


    
        
"""
queue_size = 10 ---> si tiene cola de 10, si llega a fallar el sistema guarda los ultimos 10 msj 
"""


"""
class MapBuilder_Dynamic:
    def __init__(self, Pioner3AT):
        #rospy.init_node('occupancy_grid_publisher')

        # Clase p3at para poder acceder a sus metodos
        self.p3at = Pioneer3AT()
        
        self.p3at.update_position_orientation_gz()   # Es necesaria actualizar, al llamarla sin actualizar la posición este manda un error de transformación dando posicion (0,0)

        self.xr_gz = self.p3at.pioneer_3at['position_gz']['x']   # Posicion en x del robot en el mundo gazebo
        self.yr_gz = self.p3at.pioneer_3at['position_gz']['y']   # Posicion en y del robot en el mundo gazebo
        print("**************** x_r_gz: ", self.xr_gz)
        print("**************** y_r_gz: ", self.yr_gz)

        # Fijamos la posición de odom a donde inicio el robot en el mundo gazebo
        self.ODOM_ORIGIN = (0, 0)  # Posición de origen del mapa de ocupación en el mundo gazebo, esquina superior izquierda, despues de invertir el eje y


        # Inicializar el mapa de ocupación como un array de NumPy lleno de -1, que indica áreas no exploradas.
        self.map_size_x = 500  # en celdas
        self.map_size_y = 500  # en celdas
        self.map_resolution = 0.1  # metros por celda
        
        #self.occupancy = np.full((self.map_size_x, self.map_size_y), -1, dtype=int)
        #self.occupancy_grid = np.uint8(self.occupancy[::-1]) # Invertir el mapa (matriz) para que se maneje como coordenadas de imagen

        # casteo a np.unit8 para que sea compatible con OpenCV        
        self.occupancy_grid = np.uint8(np.full((self.map_size_x, self.map_size_y), -1, dtype='int8'))
        
        # Configurar publicadores y suscriptores
        #self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.laser_sub = rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)

        # Configurar el mensaje OccupancyGrid, guardamos la info occupancy-grid en un mensaje de ROS
        self.map_msg = OccupancyGrid()
        self.map_msg.info.resolution = self.map_resolution
        self.map_msg.info.width = self.map_size_x
        self.map_msg.info.height = self.map_size_y

        # Establecer el origen del mapa a alguna posición, usamos la posición inicial del robot    
        self.map_msg.info.origin.position.x = int((self.ODOM_ORIGIN[0]) / self.map_msg.info.resolution)         # xr = 50 + m.ceil(xw/0.1)   Conversion del doc
        self.map_msg.info.origin.position.y = int((self.ODOM_ORIGIN[1]) / self.map_msg.info.resolution)         # yr = 50 - m.floor(yw/0.1)  Posicion como tipo imagen y_max, x_min
        #self.map_msg.info.origin.orientation.w = 1

        # Converción posición del robot de Gazebo a OccupancyGrid
        self.xr_grid = int((self.xr_gz - self.ODOM_ORIGIN[0]) / self.map_msg.info.resolution) 
        self.yr_grid = int((self.yr_gz - self.ODOM_ORIGIN[1]) / self.map_msg.info.resolution) + 250

    # Actualizamos la posicion del robot en el mundo gazebo
    def MAP_update_postion_robot_gz(self):
        self.p3at.update_position_orientation_gz()   # Actualizamos la posición del robot en el mundo gazebo desde la clase Pioneer3AT con el nodo tf2_ros
        # self.x_r_gz = self.p3at.pioneer_3at['position_gz']['x']   # Posicion en x del robot en el mundo gazebo
        # self.y_r_gz = self.p3at.pioneer_3at['position_gz']['y']   # Posicion en y del robot en el mundo gazebo
        print("x_r_gz: ", self.xr_gz)
        print("y_r_gz: ", self.yr_gz)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.occupancy_grid[self.yr_grid][self.xr_grid] = 254  # Actualizamos la posición del robot en el mapa de ocupación



        plt.imshow(self.occupancy_grid)
        plt.show()  # Mostrar la imagen del mapa de ocupación
    

    def laser_callback(self, msg):

        # Asumiendo que el robot está en el origen (0,0) para este ejemplo
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        coordinates = []
        coordinates_gz = []
        print('Cantidad dee rayos ',len(msg.ranges))
        
        for i, distance in enumerate(msg.ranges): # obtenemos la distancia a la que se encuentra el objeto
            # Ignorar medidas sin retorno o fuera de los límites definidos
        
            
            # comprueba si la distancia medida no es infinita. math.isinf() es una función que verifica si un número es infinito, 
            # lo cual puede ser utilizado por algunos sensores láser para indicar que no se detectó ningún objeto en esa dirección 
            # (es decir, el láser no "rebotó" en nada dentro de su rango máximo). Si la distancia es infinita, se ignora la medida.
            
            
            if not mt.isinf(distance):
                yaw_robot = self.p3at.pioneer_3at['orientation_gz']['yaw'] # = angle_min + i * angle_increment
                x_robot = self.p3at.pioneer_3at['position_gz']['x']
                y_robot = self.p3at.pioneer_3at['position_gz']['y']
                theta = angle_min + i * angle_increment
                x = distance * mt.cos(theta)
                y = distance * mt.sin(theta)

                # Calcular la posición relativa al robot.
                x_rel = distance * mt.cos(theta)
                y_rel = distance * mt.sin(theta)


                # Transformar la posición relativa al marco de referencia global.
                x_world = x_robot + (x_rel * mt.cos(yaw_robot) - y_rel * mt.sin(yaw_robot))
                y_world = y_robot + (x_rel * mt.sin(yaw_robot) + y_rel * mt.cos(yaw_robot))

                # Guardar las coordenadas globales.
                # coordinates.append((x_world, y_world))
            #     # print(f"Distancia {i}: {distance} m")
            #     # print(f"Coordenada {i}: ({x}, {y})")
                x_pos = int((x_world - self.ODOM_ORIGIN[0]) / self.map_msg.info.resolution) 
                y_pos = int((y_world - self.ODOM_ORIGIN[1]) / self.map_msg.info.resolution) + 250
                coordinates.append((x_pos, y_pos))
            #     coordinates_gz.append((x, y))


        # print('-------- ', coordinates[0])

        #for i in range(len(coordinates)):
            #cv.line(self.occupancy_grid, (self.yr_grid, self.xr_grid), coordinates[i], 0, 1)
        # cv.line(self.occupancy_grid, (self.yr_grid, self.xr_grid), coordinates[0], 0, 1)
        # cv.line(self.occupancy_grid, (self.yr_grid, self.xr_grid), (self.yr_grid, self.xr_grid + 20), 0, 1)

        for i in range(len(coordinates)):
            cv.line(self.occupancy_grid, (self.xr_grid, self.yr_grid), coordinates[i], 0, 1)
            self.occupancy_grid[coordinates[i][1]][coordinates[i][0]] = 100

        # cv.line(self.occupancy_grid, (self.xr_grid, self.yr_grid), (coordinates[0][0],coordinates[0][1]) , 0, 1)
        # cv.line(self.occupancy_grid, (self.xr_grid, self.yr_grid), coordinates[50], 0, 1)
        #print(coordinates[0])
        #print('Posicion del robot en el mapa de ocupación')
        #print( self.yr_grid, self.xr_grid)
        #cv.line(self.occupancy_grid, (50, 50), coordinates[i], 0, 1)
        self.occupancy_grid[self.yr_grid][self.xr_grid] = 100
        # Ahora `coordinates` contiene las coordenadas (x, y) de todos los puntos detectados
        # print(coordinates)

    def MAP_update_position_grid(self):
        self.MAP_update_postion_robot_gz()
        resolution = self.map_msg.info.resolution
        
        self.xr_grid = int((self.xr_gz - self.ODOM_ORIGIN[0]) / resolution) 
        self.yr_grid = int((self.yr_gz - self.ODOM_ORIGIN[1]) / resolution) + 50
        print(self.xr_gz, ' - ', self.ODOM_ORIGIN[0], ' / ', resolution)
        print(self.yr_gz, ' - ', self.ODOM_ORIGIN[1], ' / ', resolution)
        print(f"odom_origin: {self.ODOM_ORIGIN}")
        print('Posicion del robot en el mapa de ocupación')
        print(self.xr_grid)
        print(self.yr_grid)

    
    # def needs_expansion(self):
        
    #     margin = 5  # Define una margen de seguridad, por ejemplo, 5 celdas
    #     return (x < margin or x >= self.map.shape[1] - margin or
    #             y < margin or y >= self.map.shape[0] - margin)


    
    # def laser_callback(self, data):
    #     # Aquí procesarías los datos del láser y actualizarías self.occupancy_grid
    #     # Por ejemplo, podrías realizar un algoritmo de ray-casting o alguna forma de sensor fusion.

    #     # Convertir el mapa de ocupación de NumPy a un formato que pueda ser enviado por ROS
    #     self.map_msg.data = self.occupancy_grid.ravel().tolist()

    #     # Publicar el mapa de ocupación
    #     self.map_pub.publish(self.map_msg)
    
    # def run(self):
    #     rospy.spin()
"""
