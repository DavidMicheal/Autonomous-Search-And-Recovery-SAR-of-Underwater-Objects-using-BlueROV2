from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomous_rov',
            executable='video',
            name='video_node',
            output='screen',
            parameters=[{'port': 5600}],  # Update the port if needed
            remappings=[
                ('camera/image_raw', '/bluerov2/camera/image_raw')  # Remap topic if using namespaces
            ]
        )
    ])
