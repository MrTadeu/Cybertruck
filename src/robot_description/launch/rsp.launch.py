
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution



def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gazebo_ros2_control = LaunchConfiguration('use_gazebo_ros2_control', default='false')

    # Generate robot description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('robot_description'),
                 'urdf', 'robot.urdf.xacro']
            ),
            ' ',
            'use_sim_time:=', use_sim_time,
            ' ',
            'use_gazebo_ros2_control:=', use_gazebo_ros2_control
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Node for Custom Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'config',
                'robot_sim.controller_manager.yaml'
            ])
        ]
    )
    
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'config',
                'robot_sim.controller_manager.yaml'
            ])
        ],
        output='screen'
    )
    
    load_forward_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_velocity_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'config',
                'robot_sim.controller_manager.yaml'
            ])
        ],
        output='screen'
    )

    load_forward_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'forward_position_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', PathJoinSubstitution([
                FindPackageShare('robot_description'),
                'config',
                'robot_sim.controller_manager.yaml'
            ])
        ],
        output='screen'
    )



    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'use_gazebo_ros2_control',
            default_value='false',
            description='Enable ROS 2 control if true'
        ),
        node_robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_forward_velocity_controller,
        load_forward_position_controller
    ])
