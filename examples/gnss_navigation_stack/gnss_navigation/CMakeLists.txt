cmake_minimum_required(VERSION 3.5)
project(gnss_navigation)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(gnss_navigation
    src/imu_gnss_tracker.cpp
    src/pose2d.cpp
)

target_include_directories(gnss_navigation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(gnss_navigation
    Eigen3::Eigen
)

install(TARGETS gnss_navigation
    EXPORT export_gnss_navigation
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

ament_export_targets(export_gnss_navigation HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(gnss_navigation)
ament_export_dependencies(Eigen3)

# Create a config file for the package
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/gnss_navigation-config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/gnss_navigation-config.cmake"
    INSTALL_DESTINATION "lib/cmake/gnss_navigation"
)

install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/gnss_navigation-config.cmake"
    DESTINATION "lib/cmake/gnss_navigation"
)

install(EXPORT export_gnss_navigation
    FILE gnss_navigation-config-noconfig.cmake
    DESTINATION "lib/cmake/gnss_navigation"
)

ament_package()
