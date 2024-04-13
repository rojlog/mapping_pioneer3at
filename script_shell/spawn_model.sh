#!/bin/bash
# Espera hasta que Gazebo esté completamente cargado
until rosservice list | grep -q 'gazebo/spawn_urdf_model'
do
	  echo "Waiting for Gazebo to start model generation service..."
	  sleep 1
done

rosrun gazebo_ros spawn_model -param robot_description -urdf -model pioneer3at_body

