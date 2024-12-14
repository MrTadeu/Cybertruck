from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        # Joystick teleop node
        Node(
            package='teleop_mapper',
            executable='joystick_mapper.py',
            name='joystick_mapper',
            output='screen'
        )
    ])
