"""
Launch file para YOLOv8 con integración de una camara en particular.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from environs import Env

"""
To-do:
- Crear un .env para las variables de entorno.
- Crear un archivo de configuración para los argumentos de lanzamiento.
"""



    