from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    package_name = 'robot_description'

    # Incluindo o lançamento do rsp
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    pkg_path = os.path.join(get_package_share_directory(package_name))
    world_file = os.path.join(pkg_path,'config','house_world.sdf')
    robot_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')

    load_world = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )

    # Incluindo o lançamento do gz_spawn_model
    gz_spawn_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_spawn_model.launch.py'
            )
        ]),
        launch_arguments={
            'world': 'empty',
            'file': robot_file,
            'name': 'my_vehicle',
            'x': '5.0',
            'y': '5.0',
            'z': '0.5',
        }.items()
    )

    # Nó para o bridge do Gazebo
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_parameter_bridge',
        output='screen',
        arguments=['/parameter_bridge'],
    )

    return LaunchDescription([
        gazebo_bridge,
        rsp,
        load_world,
        gz_spawn_model,
    ])
