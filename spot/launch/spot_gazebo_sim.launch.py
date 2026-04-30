from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')

    robot_description_raw = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('qr_description'), 'urdf', 'spot.urdf.xacro'])
    ])
    robot_description = ParameterValue(robot_description_raw, value_type=str)

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': world,
            'on_exit_shutdown': 'True'
        }.items()
    )

    # clock_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
    #     output='screen'
    # )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'spot',
            '-string', robot_description_raw,
            '-x', '0', '-y', '0', '-z', '0.26'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        gz_sim_launch,
        rsp,
        spawn,
    ])

