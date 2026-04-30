#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # ---- args ----
    agent_num = LaunchConfiguration("agent_num")
    joy_dev = LaunchConfiguration("joy_dev")
    dry_run = LaunchConfiguration("dry_run")
    use_pwm = LaunchConfiguration("use_pwm")

    declare_args = [
        DeclareLaunchArgument(
            "agent_num",
            default_value="0",
            description="Agent number for ARS policy (0 means no agent if your code handles it)."
        ),
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device path"
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="pwm_publisher dry_run (no I2C write)"
        ),
        DeclareLaunchArgument(
            "use_pwm",
            default_value="true",
            description="Enable pwm_publisher node"
        ),
    ]

    # TODO/POINT: 패키지명이 mini_ros가 아니라 qr_control/qr_bringup이면 여기 바꿔야 함
    pkg_share = FindPackageShare("spot")

    spot_params = PathJoinSubstitution([pkg_share, "config", "spot_params.yaml"])
    policy_params = PathJoinSubstitution([pkg_share, "config", "policy_params.yaml"])
    joy_params = PathJoinSubstitution([pkg_share, "config", "joy_params.yaml"])
    servo_calib = PathJoinSubstitution([pkg_share, "config", "servo_calib.yaml"])

    # ---- nodes ----

    # spot_sm (C++)
    spot_sm_node = Node(
        package="spot",          # TODO/POINT: 실제 패키지명
        executable="spot_sm_node",     # TODO/POINT: colcon에서 빌드된 실행파일명
        name="spot_sm",
        output="screen",
        parameters=[{"frequency": 200.0}],
    )

    # joy_node (ROS2)
    # NOTE: ROS2 joy 패키지의 파라미터 이름은 joy_dev / deadzone 등이 사용됨
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="spot_joy",
        output="screen",
        parameters=[{
            "dev": joy_dev,        # 일부 배포판은 joy_dev 키를 씀. 안 되면 joy params 문서 확인 필요.
            "deadzone": 0.005,
        }],
    )

    # teleop node (C++)
    teleop_node = Node(
        package="spot",          # TODO/POINT
        executable="teleop_node",      # TODO/POINT
        name="spot_teleop",
        output="screen",
        parameters=[
            {"frequency": 200.0},
            {"axis_linear_x": 4},
            {"axis_linear_y": 3},
            {"axis_linear_z": 1},
            {"axis_angular": 0},
            {"scale_linear": 1.0},
            {"scale_angular": 1.0},
            {"button_switch": 0},
            {"button_estop": 1},
            {"debounce_thresh": 0.15},  # 원본에 있던 값
        ],
    )

    # motion control (python)
    motion_control_node = Node(
        package="spot",                # TODO/POINT: 실제 패키지명
        executable="spot_real_interface", # TODO/POINT: setup.py entrypoint or installed script명
        name="spot_real",                    # 원본 name 유지
        output="screen",
        parameters=[
            {"agent_num": agent_num},
            spot_params,
            policy_params,
        ],
    )

    # pwm_publisher (python)
    # NOTE: 조건부 실행은 launch의 IfCondition으로 하는게 정석인데,
    # 여기서는 단순성을 위해 use_pwm가 true라고 가정하고 붙임.
    pwm_node = Node(
        package="spot",                 # TODO/POINT
        executable="pwm_publisher",      # TODO/POINT: 설치된 실행 이름
        name="pwm_publisher",
        output="screen",
        parameters=[
            {"topic": "/spot/joints"},
            {"dry_run": dry_run},
            {"calib_yaml": servo_calib},
            {"angles_unit": "deg"},           # Policies가 deg publish
            {"freq": 50},
            {"address": "0x40"},
        ],
    )

    # TODO/POINT: ROS1 rosserial_python은 ROS2에서 그대로 못 씀.
    # - Teensy가 ROS2를 직접 말하게(micro-ROS) 하거나
    # - 다른 브릿지/serial node를 따로 구성해야 함.

    return LaunchDescription(
        declare_args + [
            spot_sm_node,
            joy_node,
            teleop_node,
            motion_control_node,
            pwm_node,
        ]
    )
