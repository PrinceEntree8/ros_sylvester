import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

import xacro

# Funzione richiesta da launch_ros per generare il launch file
def generate_launch_description():

    # Determino percorso del package
    pkg_path = os.path.join(get_package_share_directory('sylvester'))
    
    # Determino il file .xacro base da usare
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    
    # Processo il file .xacro in .xml
    robot_description_config = xacro.process_file(xacro_file).toxml()
    
    # Imposto i parametri per robot_state_publisher: robot_description è il file xml
    params = {'robot_description': robot_description_config}

    # Genera descrizione Launch!
    return LaunchDescription([
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params])
    ])