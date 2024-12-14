# ros2 launch robot_sim sim.launch.py use_gazebo_ros2_control:=true use_sim_time:=true
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_gazebo_ros2_control = LaunchConfiguration('use_gazebo_ros2_control', default='false')

    # Include robot state publisher (rsp) launch file
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'launch',
                'rsp.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'use_gazebo_ros2_control': use_gazebo_ros2_control}.items()
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


    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_forward_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_velocity_controller'],
        output='screen'
    )

    load_forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo_ros2_control',
            default_value='true',
            description='Enable ROS 2 control if true'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        rsp_launch,
        load_gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[gazebo_bridge, load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[load_forward_velocity_controller,
                        load_forward_position_controller],
            )
        ),
        gz_spawn_entity
    ])