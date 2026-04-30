# 최소 cmake 버전, 패키지 이름 정의
cmake_minimum_required(VERSION 3.8)
project(spot_cpp)

#의존성 패키지 찾기(라이브러리 가져오기, API, MSG)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(spot_interfaces REQUIRED)

#include 경로 include"xxx.hpp" 할 때 씀
include_directories(include)
#라이브러리 만들기
add_library(spot_cpp_lib
  src/spot.cpp
  src/teleop.cpp
)

#라이브러리에 의존성 연결 spot_cpp_lib가 다음 의존성들을 쓴다
ament_target_dependencies(spot_cpp_lib
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  spot_interfaces
)

#실행 파일 만들기 spot_sm_node 가 src/spot_sm.cpp 실행하면 된다.
add_executable(spot_sm_node src/spot_sm.cpp)
#라이브러리 연결(spot_sm_node가 spot_cpp_lib 씀)
target_link_libraries(spot_sm_node spot_cpp_lib)

ament_target_dependencies(spot_sm_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  spot_interfaces
)

add_executable(teleop_node src/teleop_node.cpp)
target_link_libraries(teleop_node spot_cpp_lib)
ament_target_dependencies(teleop_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  spot_interfaces
)
#빌드 결과를 install 폴더에 복사 -> executable not found 방지
install(TARGETS
  spot_cpp_lib
  spot_sm_node
  teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

#include 설치ㅑ헤더) 다른 페키지에서 include 가능하게
install(DIRECTORY include/
  DESTINATION include/
)

#ros2 패키지임을 알림
ament_package()
