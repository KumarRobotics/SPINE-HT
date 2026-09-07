cmake_minimum_required(VERSION 3.5)
project(spine_ht_ros)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# Include your headers
include_directories(include)

install(PROGRAMS
  scripts/twist_converter.py
  scripts/spine_node.py
  scripts/spine_ht_node.py
  scripts/nav_service_translator.py
  scripts/vlm_service_translator.py
  scripts/label_service_translator.py
  scripts/clearpath_autonomy_server.py
  scripts/spot_autonomy_server.py
  scripts/goto_nav2.py
  scripts/convert_vel_to_joy.py
  scripts/goal_frame_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, configs, and other resources if you want
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})

# Standard ament macros
ament_package()
