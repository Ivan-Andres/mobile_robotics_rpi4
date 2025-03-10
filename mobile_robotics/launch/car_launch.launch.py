import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='mobile_robotics' #<--- CHANGE ME
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux2.yaml')
    joy_params = os.path.join(get_package_share_directory(package_name),'config','joystick.yaml')
    
    # Ruta al otro launch file (ajústala según tu paquete)
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),  # Paquete donde está el otro launch
        'launch',
        'rplidar_a1_launch.py'  # Nombre del launch file a incluir
    )

    joy_node = Node(package='joy', 
                    executable='joy_node',
                    parameters=[joy_params],
    )

    comand_joy_node = Node(
        package=package_name,  # Reemplaza 'tu_paquete' con el nombre de tu paquete
        executable='comand_joy',  # Nombre del ejecutable de tu nodo
        output='screen',
    )

    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params],
                    remappings=[('/cmd_vel_out','/cmd_vel')]
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    car_node = Node(package='car_control', 
                    executable='car_control',
                    output='screen',
    )

    lidar_aebs=Node(
        package=package_name,  # Reemplaza 'tu_paquete' con el nombre de tu paquete
        executable='AEBS_node',  # Nombre del ejecutable de tu nodo
        output='screen'  # Esto hace que el nodo imprima en la consola
    )

    # Launch them all!
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        ),
        lidar_aebs,
        joy_node,
        comand_joy_node,
        twist_mux_node,
        car_node,
    ])