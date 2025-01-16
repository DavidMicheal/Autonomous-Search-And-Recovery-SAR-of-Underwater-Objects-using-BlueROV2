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
            namespace='bluerov2',  # Uncomment if using a namespace
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='bluerov2',
            output='screen',
            parameters=[
                {'dev': '/dev/input/js0'},
                {'deadzone': 0.2},
                {'autorepeat_rate': 0.0}
            ],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            namespace='bluerov2',
            output='screen',
            parameters=[
                {'require_enable_button': False},
                {'axis_linear.x': 1},
                {'axis_linear.y': 0},
                {'axis_linear.z': 4},
                {'axis_angular.yaw': 3},
                {'axis_angular.roll': 7},
                {'axis_angular.pitch': 6},
                {'scale_linear.x': 0.7},
                {'scale_linear.y': 0.7},
                {'scale_linear.z': 0.7},
                {'scale_angular.yaw': 0.4},
                {'scale_angular.roll': 0.2},
                {'scale_angular.pitch': 0.2},
            ],
        ),
        Node(
            package='autonomous_rov',
            executable='video',
            name='video_node',
            namespace='bluerov2',
            output='screen',
        ),
        Node(
            package='autonomous_rov',
            executable='listenerMIR',
            name='listenerMIR',
            namespace='bluerov2',
            output='screen',
        ),
    ])
