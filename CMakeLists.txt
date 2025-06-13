cmake_minimum_required(VERSION 3.8)
project(husky_kinova_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_bridge REQUIRED)


INSTALL(DIRECTORY
  launch meshes worlds description  
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
