cmake_minimum_required(VERSION 3.16)
project(px4ctrl)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(px4ctrl_node src/px4ctrl_node.cpp)
target_compile_features(px4ctrl_node PUBLIC cxx_std_17)
target_link_libraries(px4ctrl_node Eigen3::Eigen)
ament_target_dependencies(px4ctrl_node
  geometry_msgs nav_msgs px4_msgs quadrotor_msgs rclcpp std_msgs)

install(TARGETS px4ctrl_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/manual_goal.py DESTINATION lib/${PROJECT_NAME} RENAME manual_goal)
install(PROGRAMS scripts/offboard_thrust_probe.py DESTINATION lib/${PROJECT_NAME} RENAME offboard_thrust_probe)
install(PROGRAMS scripts/track_eval.py DESTINATION lib/${PROJECT_NAME} RENAME track_eval)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
