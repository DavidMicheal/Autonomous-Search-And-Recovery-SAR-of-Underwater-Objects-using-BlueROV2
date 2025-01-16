from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # MavlinkPressureNode
    mavlink_pressure_node = Node(
        package='mavlink_pressure_node',  # Replace with your package name
        executable='pressure_node',  # Matches entry point in setup.py
        name='mavlink_pressure_node',
        parameters=[{'port': 14555}],  # Pass the UDP port parameter
        output='screen',
    )
    
    return LaunchDescription([
        mavlink_pressure_node,
    ])
