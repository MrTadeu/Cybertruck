
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Include robot state publisher (rsp) launch file
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'rsp.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'false'}.items()
    )

    # Spawn robot entity in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 
                   'robot_description',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.17',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                    '-name',
                    'robot',
                    '-allow_renaming',
                    'true']
    )

    # Bridge for Gazebo and ROS2
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )

    # Launch Gazebo environment
    load_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    return LaunchDescription([
        rsp_launch,
        load_gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[gazebo_bridge],
            )
        ),
        gz_spawn_entity
    ])
