from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://@192.168.2.1:14550'},
                {'fcu_protocol': 'v2.0'},
            ],
            # namespace='bluerov2',  # Uncomment if using a namespace
        ),
    ])
