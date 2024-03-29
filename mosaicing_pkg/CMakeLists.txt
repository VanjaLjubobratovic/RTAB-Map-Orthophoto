cmake_minimum_required(VERSION 3.8)
project(mosaicing_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(rtabmap_msgs REQUIRED)



add_executable(mosaicing_node 
    src/mosaicing_node.cpp 
    src/mosaicing_tools.cpp
    src/mosaicing_tools.h
)


target_include_directories(mosaicing_node PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${rtabmap_conversions_INCLUDE_DIRS}
    ${rtabmap_msgs_INCLUDE_DIRS}
)

# Link your executable with the necessary libraries
ament_target_dependencies(mosaicing_node
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    PCL
    OpenCV
    rtabmap_conversions
    rtabmap_msgs
)

target_link_libraries(mosaicing_node
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${pcl_conversions_LIBRARIES}
)

# Install the executable
install(TARGETS
    mosaicing_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
