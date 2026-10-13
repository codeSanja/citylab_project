cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "src/custom_interfaces/srv/GetDirection.srv"
        DEPENDENCIES sensor_msgs
)

add_executable(patrol_node src/patrol.cpp)
ament_target_dependencies(patrol_node
        rclcpp
        sensor_msgs
        cv_bridge
        tf2
        nav_msgs
        std_srvs
        geometry_msgs
)

install(TARGETS patrol_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})


# Add the executable
add_executable(direction_service_executable src/direction_service.cpp)

ament_target_dependencies(direction_service_executable
        rclcpp
        std_srvs
)

rosidl_get_typesupport_target(
        cpp_typesupport_target
        ${PROJECT_NAME}
        "rosidl_typesupport_cpp"
)

target_link_libraries(direction_service_executable
        "${cpp_typesupport_target}"
)

target_link_libraries(patrol_node
        "${cpp_typesupport_target}"
)

# Install the executable
install(TARGETS
        direction_service_executable
        DESTINATION lib/${PROJECT_NAME}
)


# Add the executable
add_executable(test_service_executable src/test_service.cpp)

ament_target_dependencies(test_service_executable
        rclcpp
        std_srvs
)

target_link_libraries(test_service_executable
        "${cpp_typesupport_target}"
)

target_link_libraries(test_service_executable
        "${cpp_typesupport_target}"
)

# Install the executable
install(TARGETS
        test_service_executable
        DESTINATION lib/${PROJECT_NAME}
)


add_executable(patrol_with_service_node_executable src/patrol_with_service.cpp)

ament_target_dependencies(patrol_with_service_node_executable
        rclcpp
        sensor_msgs
        cv_bridge
        tf2
        nav_msgs
        std_srvs
        geometry_msgs
)

install(TARGETS patrol_with_service_node_executable DESTINATION lib/${PROJECT_NAME})

target_link_libraries(patrol_with_service_node_executable
        "${cpp_typesupport_target}"
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
