cmake_minimum_required(VERSION 3.8)
project(gnss_navigation_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gnss_navigation REQUIRED)

# Add the executable for the ROS node
add_executable(imu_gnss_tracker_ros src/imu_gnss_tracker_ros.cpp)

target_include_directories(imu_gnss_tracker_ros PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link the executable against its dependencies
ament_target_dependencies(imu_gnss_tracker_ros
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

target_link_libraries(imu_gnss_tracker_ros
  gnss_navigation
)

# Install the executable
install(TARGETS imu_gnss_tracker_ros
  DESTINATION lib/${PROJECT_NAME}
)

# Install helper scripts
install(PROGRAMS
  scripts/gnss_spoofer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and param files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
