cmake_minimum_required(VERSION 3.8)
project(spot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# msg 파일들 나열 (실제 파일명과 정확히 일치해야 함)
set(msg_files
  "msg/MiniCmd.msg"
  "msg/JoyButtons.msg"
  "msg/JointAngles.msg"
  "msg/JointPulse.msg"
  "msg/ContactData.msg"
  "msg/AgentData.msg"
  "msg/IMUdata.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
