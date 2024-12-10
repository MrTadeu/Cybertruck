from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_ros2_control = LaunchConfiguration('use_ros2_control', default='false')

    # Generate robot description from xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindPackageShare('robot_description'), 'urdf', 'robot.urdf.xacro']),
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time
    ])
    robot_description = {'robot_description': robot_description_content}
    
    
    #robot_description_content = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        PathJoinSubstitution([
    #            FindPackageShare('robot_description'), 
    #            'launch', 
    #            'rsp.launch.py'
    #        ])
    #    ),
    #    launch_arguments={
    #        'use_sim_time': 'true',
    #        'use_ros2_control': 'false' 
    #    }.items()
    #)
    #robot_description = {'robot_description': robot_description_content}

    # Robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Enable ROS 2 control if true'
        ),
        node_robot_state_publisher
    ])
