import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    package_name='wall_following'

    car_launch = os.path.join(
        get_package_share_directory('mobile_robotics'),  # Paquete donde está el otro launch
        'launch',
        'car_launch.launch.py'  # Nombre del launch file a incluir
    )

    dist_finder = Node(
        package=package_name,
        executable='dist_finder_alpha', 
        output='screen', 
        parameters=[{'use_sim_time': True}]  
    )

    controller = Node(
        package=package_name,  
        executable='control_alpha',  
        output='screen',  
        parameters=[{'use_sim_time': True}] 
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(car_launch)
        ),
        dist_finder,
        controller,
    ])