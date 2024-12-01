from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os

import xacro

def generate_launch_description():
    """ urdf_xacro_file = os.path.join(
        os.path.dirname(get_package_prefix('robot_description')), '..', 'src', 'robot_description', 'urdf', 'robot.urdf.xacro'
    ) """
    package_share_directory = get_package_share_directory('robot_description')
    urdf_xacro_file = os.path.join(package_share_directory, 'urdf', 'robot.urdf.xacro')
    
    urdf_file_content = xacro.process_file(urdf_xacro_file).toxml()

    """ ros2 run joint_state_publisher_gui joint_state_publisher_gui """
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file_content}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])