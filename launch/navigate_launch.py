from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_navigator',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
        Node(
            package='simple_navigator',
            executable='simple_navigator', 
            name='simple_navigator',
            output='screen'
        )
    ])