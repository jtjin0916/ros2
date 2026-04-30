#qr프로젝트 상의 새로운 형식으로 만든 rviz 파일 런치 파일
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_pkg = 'qr_description'   # 네 패키지명에 맞게
    bringup_pkg = 'qr_bringup'

    urdf_file = os.path.join(
        get_package_share_directory(desc_pkg),
        'urdf',
        'qr.urdf'                 # xacro면 qr.urdf.xacro로 바꾸고 Command 쓰는 게 정석
    )

    rviz_config = os.path.join(
        get_package_share_directory(bringup_pkg),
        'rviz',
        'qr.rviz'                 # 너가 File 기반으로 저장해둔 rviz
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
