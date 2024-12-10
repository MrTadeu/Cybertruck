from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
import os
from launch.actions import DeclareLaunchArgument

import xacro

def generate_launch_description():
    """ package_share_directory = get_package_share_directory('robot_description')
    urdf_xacro_file = os.path.join(package_share_directory, 'urdf', 'robot.urdf.xacro')
    
    urdf_file_content = xacro.process_file(urdf_xacro_file).toxml() """

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    print(xacro_file)
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])

    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params]
        )
    
    """ ros2 run joint_state_publisher_gui joint_state_publisher_gui """

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Use ros2_control if true'),
        
           node_robot_state_publisher 

    ])