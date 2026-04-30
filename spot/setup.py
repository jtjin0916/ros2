from setuptools import find_packages, setup
#패키지 이름
package_name = 'spot'

setup(
    name=package_name,
    version='0.0.0',
    #spot 폴더 안에 .py 파일을 패키지로 등록 __init__.py필요 / exclude : 제외
    packages=find_packages(exclude=['test']),
    #데이터 파일 위치(rosource/spot: 패키지 존재 표시, package.xml: 메타 정보 ->ros2 run)
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spot_move.launch.py']),
    ],
    #필수 파이썬 의존성 pip install 시 같이 설치 됨(하지만 package.xml에서 주로 관리)
    install_requires=['setuptools'],
    #압축해서 써도 되는지
    zip_safe=True,
    #패키지 관리자
    maintainer='user',
    maintainer_email='user@todo.todo',
    #패키지 설명
    description='TODO: Package description',
    #라이센스
    license='TODO: License declaration',
    #선택적 의존성 pip install spot[test] 명령어로 씀
    extras_require={
        'test': [
            'pytest',
        ],
    },
    #실행이름 =모듈경로 : 함수 (ros2 run spot spot_real_interface : main() 실행)
    entry_points={
        'console_scripts': [
            'spot_real_interface = spot.spot_real_interface:main',
            'pwm_publisher = spot.pwm_publisher:main',
            'spot_pybullet_interface = spot.qr_pybullet_interface:main',
            'imu_publisher = spot.imu_publisher:main'
        ],
    },
)
