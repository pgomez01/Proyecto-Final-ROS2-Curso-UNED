cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Navigate.action"
)
#Instalar base node
add_executable(base_node src/base_node.cpp)
ament_target_dependencies(base_node rclcpp tf2_ros geometry_msgs rclcpp_action std_msgs) 

#Instalar controller node
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp rclcpp_action)

#Instalar security node
add_executable(security_node src/security_node.cpp)
ament_target_dependencies(security_node rclcpp sensor_msgs std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_link_libraries(base_node "${cpp_typesupport_target}")
target_link_libraries(controller_node "${cpp_typesupport_target}")

install(TARGETS
  base_node
  controller_node
  security_node
  DESTINATION lib/${PROJECT_NAME}
)

#Instalar carpeta Launch
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
