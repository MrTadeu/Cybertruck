from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_mapper',
            executable='teleop_mapper.py',
            name='teleop_mapper',
            output='screen'
        )
    ])
