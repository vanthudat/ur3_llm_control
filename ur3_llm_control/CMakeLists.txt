cmake_minimum_required(VERSION 3.8)
project(ur3_llm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(robot_skills src/robot_skills.cpp)
target_include_directories(robot_skills PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_skills
  control_msgs
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  ros_gz_interfaces
  sensor_msgs
  shape_msgs
  trajectory_msgs
)

add_executable(skill_executor src/skill_executor.cpp)
target_link_libraries(skill_executor robot_skills nlohmann_json::nlohmann_json)
ament_target_dependencies(skill_executor rclcpp std_msgs)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python/${PROJECT_NAME})

install(PROGRAMS scripts/llm_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME llm_planner
)
install(PROGRAMS scripts/command.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME command
)
install(DIRECTORY config launch srdf urdf worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS robot_skills skill_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(task_validator_test test/test_task_validator.py)
endif()

ament_package()
