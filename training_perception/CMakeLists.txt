cmake_minimum_required(VERSION 3.8)
project(training_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(training_msgs REQUIRED)

include_directories(include)

set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)  #指定pcl1.8路径

include_directories(${PCL_INCLUDE_DIRS})

add_executable(euclidean_clustering training_perception/euclidean_clustering_node.cpp)
ament_target_dependencies(euclidean_clustering rclcpp sensor_msgs geometry_msgs visualization_msgs pcl_conversions PCL training_msgs)
target_link_libraries(euclidean_clustering ${PCL_LIBRARIES})

add_executable(lidar_filter training_perception/lidar_filter_node.cpp)
ament_target_dependencies(lidar_filter rclcpp tf2_ros pcl_ros sensor_msgs pcl_conversions PCL)
target_link_libraries(lidar_filter ${PCL_LIBRARIES})


install(TARGETS
  lidar_filter
  euclidean_clustering
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  # for python files
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()