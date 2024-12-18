from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

# open a new terminal and run   ros2 run teleop_twist_keyboard teleop_twist_keyboard

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='\033[33mAbra um novo terminal e execute:\033[m \033[34;1mros2 run teleop_twist_keyboard teleop_twist_keyboard\033[m'),
        Node(
            package='teleop_mapper',
            executable='teleop_mapper.py',
            name='teleop_mapper',
            output='screen',
        )
    ])
