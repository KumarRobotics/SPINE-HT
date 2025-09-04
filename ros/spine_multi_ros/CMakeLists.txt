cmake_minimum_required(VERSION 3.5)
project(spine_multi_ros)

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

# Declare your C++ executable(s)
# add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)

# Link dependencies
# ament_target_dependencies(odom_tf_broadcaster
# rclcpp
# nav_msgs
# geometry_msgs
# tf2_ros
# )

install(PROGRAMS
  scripts/twist_converter.py
  scripts/spine_node.py
  scripts/spine_multi_node.py
  scripts/nav_service_translator.py
  scripts/vlm_service_translator.py
  scripts/label_service_translator.py
  scripts/jackal_autonomy_server.py
  scripts/spot_autonomy_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the executable
# install(TARGETS odom_tf_broadcaster
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files, configs, and other resources if you want
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)

# ament_export_dependencies(rosidl_default_runtime)
# ament_python_install_package(${PROJECT_NAME} DEPENDS ${PROJECT_NAME}_generate_interfaces)

ament_python_install_package(${PROJECT_NAME})

# Standard ament macros
ament_package()


# cmake_minimum_required(VERSION 3.8)
# project(spine_multi_ros)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)

# find_package(rclcpp REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

# find_package(rosidl_default_generators REQUIRED)

# # Custom service definition (optional)
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/Mission.srv"
#   DEPENDENCIES std_msgs
# )

# # ament_python_install_package(${PROJECT_NAME})

# add_executable(odom_tf_broadcaster 
#   src/odom_tf_broadcaster.cpp
# )

# ament_target_dependencies(odom_tf_broadcaster
#   rclcpp
#   nav_msgs
#   geometry_msgs
#   tf2_ros
# )

# # install(PROGRAMS
# #   scripts/twist_converter.py
# #   src/python/nav_client.py
# #   scripts/spine_node.py
# #   DESTINATION lib/${PROJECT_NAME}
# # )

# # Install executable
# install(TARGETS
#   odom_tf_broadcaster
#   DESTINATION lib/${PROJECT_NAME}
# )


# # install(DIRECTORY
# #   launch 
# #   config
# #   DESTINATION share/${PROJECT_NAME}) 

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()
