from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    paused = LaunchConfiguration('paused')
    debug = LaunchConfiguration('debug')

    # xacro -> robot_description
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('qr_description'), 'urdf', 'spot.urdf.xacro'])
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'gui': gui,
            'verbose': debug,
            # paused/headless는 gazebo.launch.py 버전에 따라 인자명이 다를 수 있음
            # 안 먹으면 아래처럼 world/extra_gazebo_args로 처리해야 함
        }.items()
    )

    # robot_state_publisher (URDF TF 발행)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    # Spawn entity into Gazebo using robot_description topic
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'spot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.26'
        ],
        output='screen'
    )

    # (네 ROS1에서 include하던 controller.launch 역할)
    # ROS2에서는 보통 controller_manager spawner 노드들을 실행함
    # 아래는 예시 (패키지/컨트롤러 이름은 너 설정에 맞춰 바꿔야 함)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 예: position controller / trajectory controller 중 하나
    # position_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['position_controller', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('paused', default_value='true'),
        DeclareLaunchArgument('debug', default_value='false'),

        gazebo_launch,
        rsp,
        spawn,

        joint_state_broadcaster_spawner,
        # position_controller_spawner,
    ])
