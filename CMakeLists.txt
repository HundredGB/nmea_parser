cmake_minimum_required(VERSION 3.5)
project(nmea_parser)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gpsx.msg"
)

add_executable(pub_node src/node.cpp)
add_executable(sub_node src/gga_subsnode.cpp)
ament_target_dependencies(pub_node rclcpp std_msgs sensor_msgs)
ament_target_dependencies(sub_node rclcpp std_msgs sensor_msgs)
rosidl_target_interfaces(pub_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(sub_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(pub_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_include_directories(sub_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  pub_node
  sub_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  INCLUDES
  DESTINATION include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
