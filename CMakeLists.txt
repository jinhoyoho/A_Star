cmake_minimum_required(VERSION 3.5)
project(my_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(example_interfaces REQUIRED)

# Declare a library
add_library(${PROJECT_NAME}
  src/math_helper.cpp
  src/nodes.cpp
  src/a_star.cpp
  src/global_planner.cpp
  src/dwa_planner.cpp
)

include_directories(include
 ${nav_msgs_INCLUDE_DIRS}
 ${nav2_core_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS})


# 헤더 파일 경로 추가
target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Declare an executable for my_node
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs sensor_msgs geometry_msgs PCL pcl_conversions)
target_link_libraries(my_node ${PCL_LIBRARIES})


add_executable(main src/main.cpp src/a_star.cpp src/nodes.cpp src/global_planner.cpp src/math_helper.cpp src/dwa_planner.cpp)
ament_target_dependencies(main rclcpp nav2_costmap_2d geometry_msgs visualization_msgs PCL pcl_conversions)
target_link_libraries(main ${PCL_LIBRARIES})

add_executable(relocalization src/relocalization.cpp)
ament_target_dependencies(relocalization rclcpp std_msgs sensor_msgs PCL pcl_conversions visualization_msgs)
target_link_libraries(relocalization ${PCL_LIBRARIES})

add_executable(map2d src/map2d.cpp)
ament_target_dependencies(map2d rclcpp std_msgs sensor_msgs PCL pcl_conversions)
target_link_libraries(map2d ${PCL_LIBRARIES})

add_executable(lidar src/lidar_proccesing.cpp)
ament_target_dependencies(lidar rclcpp std_msgs sensor_msgs PCL pcl_conversions)
target_link_libraries(lidar ${PCL_LIBRARIES})

# Link dependencies to the library
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs
  visualization_msgs
  PCL 
  pcl_conversions
)

# Install targets
install(TARGETS
  main
  relocalization
  map2d
  lidar
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

