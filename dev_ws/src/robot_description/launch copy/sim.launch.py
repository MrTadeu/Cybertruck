from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'robot_description'

    # Include robot state publisher (rsp) launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(package_name), 
                'launch', 
                'rsp.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'false'
        }.items()
    )

    # World file path
    world_file = PathJoinSubstitution([
        FindPackageShare(package_name), 
        'config', 
        'house_world.sdf'
    ])

    # Robot file path
    robot_file = PathJoinSubstitution([
        FindPackageShare(package_name), 
        'urdf', 
        'robot.urdf.xacro'
    ])

    # Load world into Gazebo simulation
    #load_world = ExecuteProcess(
    #    cmd=['gz', 'sim', world_file],
    #    output='screen'
    #)
    load_world = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])


    # Include Gazebo model spawner launch file
    #gz_spawn_model = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        PathJoinSubstitution([
    #            FindPackageShare('ros_gz_sim'), 
    #            'launch', 
    #            'gz_spawn_model.launch.py'
    #        ])
    #    ),
    #    launch_arguments={
    #        'world': 'house',
    #        'file': robot_file,
    #        'name': 'my_vehicle',
    #        'x': '5.0',
    #        'y': '5.0',
    #        'z': '0.5',
    #    }.items()
    #)
    gz_spawn_modelgz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'ackermann', '-allow_renaming', 'true'],
    )
    

    # Node for Gazebo-ROS bridge
    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    )


    return LaunchDescription([
        rsp,
        load_world,
        gz_spawn_model,
        gazebo_bridge,
    ])
