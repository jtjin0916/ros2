from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Launch args
    frequency = LaunchConfiguration("frequency")
    joy_dev = LaunchConfiguration("joy_dev")
    deadzone = LaunchConfiguration("deadzone")

    axis_linear_x = LaunchConfiguration("axis_linear_x")
    axis_linear_y = LaunchConfiguration("axis_linear_y")
    axis_linear_z = LaunchConfiguration("axis_linear_z")
    axis_angular = LaunchConfiguration("axis_angular")

    scale_linear = LaunchConfiguration("scale_linear")
    scale_angular = LaunchConfiguration("scale_angular")

    button_switch = LaunchConfiguration("button_switch")
    button_estop = LaunchConfiguration("button_estop")
    rb = LaunchConfiguration("rb")
    lb = LaunchConfiguration("lb")
    rt = LaunchConfiguration("rt")
    lt = LaunchConfiguration("lt")
    updown = LaunchConfiguration("updown")
    leftright = LaunchConfiguration("leftright")
    scale_bumper = LaunchConfiguration("scale_bumper")
    debounce_thresh = LaunchConfiguration("debounce_thresh")

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("frequency", default_value="200.0"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("deadzone", default_value="0.05"),

        DeclareLaunchArgument("axis_linear_x", default_value="3"),
        DeclareLaunchArgument("axis_linear_y", default_value="2"),
        DeclareLaunchArgument("axis_linear_z", default_value="1"),
        DeclareLaunchArgument("axis_angular", default_value="0"),

        DeclareLaunchArgument("scale_linear", default_value="1.0"),
        DeclareLaunchArgument("scale_angular", default_value="1.0"),

        DeclareLaunchArgument("button_switch", default_value="1"),
        DeclareLaunchArgument("button_estop", default_value="2"),
        DeclareLaunchArgument("rb", default_value="9"),
        DeclareLaunchArgument("lb", default_value="8"),
        DeclareLaunchArgument("rt", default_value="7"),
        DeclareLaunchArgument("lt", default_value="6"),
        DeclareLaunchArgument("updown", default_value="5"),
        DeclareLaunchArgument("leftright", default_value="4"),
        DeclareLaunchArgument("scale_bumper", default_value="1.0"),
        DeclareLaunchArgument("debounce_thresh", default_value="0.15"),

        # 1) State Machine Node (ROS1: pkg=mini_ros type=spot_sm)
        Node(
            package="spot_cpp",               # TODO: ROS2 패키지명으로 교체
            executable="spot_sm_node",             # TODO: ROS2 실행파일명으로 교체(보통 *_node)
            name="spot_sm",
            output="screen",
            parameters=[{
                "frequency": frequency,
            }],
        ),

        # 2) Joystick Node (ROS2: joy 패키지의 joy_node)
        Node(
            package="joy",
            executable="joy_node",
            name="spot_joy",
            output="screen",
            respawn=True,
            parameters=[{
                "dev": joy_dev,
                "deadzone": deadzone,
            }],
        ),

        # 3) Teleop Node (ROS1: pkg=mini_ros type=teleop_node)
        Node(
            package="spot_cpp",               # TODO: ROS2 패키지명으로 교체
            executable="teleop_node",         # TODO: ROS2 실행파일명으로 교체
            name="spot_teleop",
            output="screen",
            parameters=[{
                "frequency": frequency,
                "axis_linear_x": axis_linear_x,
                "axis_linear_y": axis_linear_y,
                "axis_linear_z": axis_linear_z,
                "axis_angular": axis_angular,
                "scale_linear": scale_linear,
                "scale_angular": scale_angular,
                "scale_bumper": scale_bumper,
                "button_switch": button_switch,
                "button_estop": button_estop,
                "rb": rb,
                "lb": lb,
                "rt": rt,
                "lt": lt,
                "updown": updown,
                "leftright": leftright,
                "debounce_thresh": debounce_thresh,
            }],
        ),

        # 4) Policy / PyBullet Interface Node (ROS1: type=spot_pybullet_interface)
        #Node(
        #    package="qr_control",                 # TODO: ROS2 패키지명으로 교체
        #    executable="qr_pybullet_interface",  # TODO: ROS2 실행파일명으로 교체
        #    name="spot_pybullet",
        #    output="screen",
        #),
        ExecuteProcess(
            cmd=[
                '/home/user/ros2_ws/.venv/bin/python',  # venv python 경로
                '-m', 'spot.spot_pybullet_interface'
            ],
            output='screen'
        )
    ])
